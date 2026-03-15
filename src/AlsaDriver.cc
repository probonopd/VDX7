/**
 *  VDX7 - Virtual DX7 synthesizer emulation
 *  Copyright (C) 2023  chiaccona@gmail.com 
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <algorithm>
#include <sched.h>

#include "AlsaDriver.h"

AlsaDriver::~AlsaDriver() {
	running = false;
	if (audio_thread.joinable()) audio_thread.join();
	if (midi_encoder) { snd_midi_event_free(midi_encoder); midi_encoder = nullptr; }
	if (seq) { snd_seq_close(seq); seq = nullptr; }
	if (pcm_handle) { snd_pcm_drain(pcm_handle); snd_pcm_close(pcm_handle); pcm_handle = nullptr; }
	capClose();
}

// ── WAV capture helpers ────────────────────────────────────────────────────────

void AlsaDriver::openCapture(const char *wavfile, float wavSeconds) {
	cap_fp = fopen(wavfile, "wb");
	if (!cap_fp) {
		fprintf(stderr, "AlsaDriver: cannot open capture file '%s'\n", wavfile);
		return;
	}
	cap_samples = 0;
	cap_limit   = (wavSeconds > 0.f) ? (int32_t)(wavSeconds * 48000.f) : -1;
	// Placeholder 44-byte WAV header; fixed in capClose()
	uint8_t hdr[44] = {};
	fwrite(hdr, 1, 44, cap_fp);
	fprintf(stderr, "AlsaDriver: capturing to '%s'%s\n", wavfile,
	        cap_limit > 0 ? " (time-limited)" : "");
}

void AlsaDriver::capWrite(const float *buf, int nframes) {
	if (!cap_fp) return;
	for (int i = 0; i < nframes; i++) {
		if (cap_limit > 0 && cap_samples >= cap_limit) { capClose(); return; }
		float s = buf[i];
		if (s >  1.0f) s =  1.0f;
		if (s < -1.0f) s = -1.0f;
		int16_t v = (int16_t)(s * 32767.0f);
		fwrite(&v, 2, 1, cap_fp);
		++cap_samples;
	}
}

void AlsaDriver::capClose() {
	if (!cap_fp) return;
	uint32_t dataSize = (uint32_t)cap_samples * 2u;
	uint32_t fileSize = dataSize + 36u;
	fseek(cap_fp, 0, SEEK_SET);
	fwrite("RIFF",  1, 4, cap_fp); fwrite(&fileSize, 4, 1, cap_fp);
	fwrite("WAVE",  1, 4, cap_fp);
	fwrite("fmt ",  1, 4, cap_fp);
	uint32_t fmtSz = 16;   fwrite(&fmtSz, 4, 1, cap_fp);
	uint16_t fmt   =  1;   fwrite(&fmt,   2, 1, cap_fp); // PCM
	uint16_t ch    =  1;   fwrite(&ch,    2, 1, cap_fp); // mono
	uint32_t sr    = 48000; fwrite(&sr,   4, 1, cap_fp);
	uint32_t br    = sr*2; fwrite(&br,    4, 1, cap_fp);
	uint16_t ba    =  2;   fwrite(&ba,    2, 1, cap_fp);
	uint16_t bps   = 16;   fwrite(&bps,   2, 1, cap_fp);
	fwrite("data",  1, 4, cap_fp); fwrite(&dataSize, 4, 1, cap_fp);
	fclose(cap_fp);
	cap_fp = nullptr;
	fprintf(stderr, "AlsaDriver: capture closed (%d samples, %.1f sec)\n",
	        cap_samples, cap_samples / 48000.0f);
}

// Open ALSA sequencer and create a single duplex MIDI port
int AlsaDriver::openSeq() {
	int err;
	if ((err = snd_seq_open(&seq, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK)) < 0) {
		fprintf(stderr, "ALSA: cannot open sequencer: %s\n", snd_strerror(err));
		return 1;
	}
	snd_seq_set_client_name(seq, "VDX7");
	our_client = snd_seq_client_id(seq);

	seq_port = snd_seq_create_simple_port(seq, "VDX7 MIDI",
		SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ |
		SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
		SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
	if (seq_port < 0) {
		fprintf(stderr, "ALSA: cannot create sequencer port: %s\n", snd_strerror(seq_port));
		return 1;
	}

	// MIDI encoder for converting raw bytes to ALSA seq events (TX path)
	if ((err = snd_midi_event_new(4096, &midi_encoder)) < 0) {
		fprintf(stderr, "ALSA: cannot create MIDI encoder: %s\n", snd_strerror(err));
		return 1;
	}
	snd_midi_event_init(midi_encoder);

	fprintf(stderr, "ALSA MIDI: client %d port %d\n", our_client, seq_port);
	return 0;
}

// Open ALSA PCM for stereo float (or S16) playback
int AlsaDriver::openPCM(const char *dev_name) {
	int err;
	if (!dev_name || !*dev_name) dev_name = "default";

	// ── Try to open the device and negotiate stereo (2 channels). ──
	// Some USB audio devices (e.g. Bose USB Audio) expose 6+ channels
	// on their raw hw: interface.  Writing our mono signal into 6
	// interleaved channels and relying on the device to route only FL/FR
	// is unreliable – many USB audio DSPs garble or mute when they
	// receive unexpected channel layouts.
	//
	// Strategy:
	//   1. Open the device and try to get exactly 2 channels.
	//   2. If the device forces more (set_channels_near rounds up),
	//      close it and reopen through ALSA's "plug:" conversion layer,
	//      which does proper channel mapping/down-mix transparently.
	//   3. If plug also fails, fall back to the original device with
	//      whatever channel count the hardware requires.

	auto try_open = [&](const char *name) -> int {
		if ((err = snd_pcm_open(&pcm_handle, name,
					SND_PCM_STREAM_PLAYBACK, 0)) < 0)
			return err;
		return 0;
	};

	auto try_configure = [&](const char *name) -> int {
		snd_pcm_hw_params_t *params;
		snd_pcm_hw_params_alloca(&params);
		snd_pcm_hw_params_any(pcm_handle, params);

		if ((err = snd_pcm_hw_params_set_access(pcm_handle, params,
					SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
			fprintf(stderr, "ALSA: cannot set interleaved access: %s\n", snd_strerror(err));
			return 1;
		}

		// Try 32-bit float first, fall back to signed 16-bit
		use_float = true;
		if ((err = snd_pcm_hw_params_set_format(pcm_handle, params,
					SND_PCM_FORMAT_FLOAT)) < 0) {
			use_float = false;
			if ((err = snd_pcm_hw_params_set_format(pcm_handle, params,
						SND_PCM_FORMAT_S16_LE)) < 0) {
				fprintf(stderr, "ALSA: cannot set sample format: %s\n", snd_strerror(err));
				return 1;
			}
		}

		// Negotiate channel count — prefer 2, accept 1
		n_channels = 2;
		if ((err = snd_pcm_hw_params_set_channels_near(pcm_handle, params, &n_channels)) < 0) {
			fprintf(stderr, "ALSA: cannot set channel count: %s\n", snd_strerror(err));
			return 1;
		}

		// If we got more than 2 channels, signal the caller to try plug:
		if (n_channels > 2) return -1;

		unsigned int rate = 48000;
		if ((err = snd_pcm_hw_params_set_rate_near(pcm_handle, params, &rate, nullptr)) < 0) {
			fprintf(stderr, "ALSA: cannot set sample rate: %s\n", snd_strerror(err));
			return 1;
		}
		synth->setSampleRate(rate);

		// Period = BufSize frames; buffer = 4 periods
		snd_pcm_uframes_t period = BufSize;
		snd_pcm_hw_params_set_period_size_near(pcm_handle, params, &period, nullptr);
		snd_pcm_uframes_t buffer_size = period * 4;
		snd_pcm_hw_params_set_buffer_size_near(pcm_handle, params, &buffer_size);

		if ((err = snd_pcm_hw_params(pcm_handle, params)) < 0) {
			fprintf(stderr, "ALSA: cannot set PCM hw params: %s\n", snd_strerror(err));
			return 1;
		}
		snd_pcm_prepare(pcm_handle);

		snd_pcm_uframes_t actual_period = 0;
		snd_pcm_hw_params_get_period_size(params, &actual_period, nullptr);
		period_frames = (int)actual_period;

		snd_pcm_uframes_t actual_buf = 0;
		snd_pcm_hw_params_get_buffer_size(params, &actual_buf);

		fprintf(stderr, "ALSA PCM: device='%s' rate=%u format=%s channels=%u "
			"period=%d buffer=%lu\n",
			name, rate, use_float ? "FLOAT" : "S16_LE", n_channels,
			period_frames, (unsigned long)actual_buf);
		return 0;
	};

	// --- Attempt 1: open the device as given ---
	if (try_open(dev_name) < 0) {
		fprintf(stderr, "ALSA: cannot open PCM '%s': %s\n", dev_name, snd_strerror(err));
		return 1;
	}
	int rc = try_configure(dev_name);
	if (rc == 0) goto configured; // got ≤2 channels, we're done

	// --- Attempt 2: device wants >2 channels; reopen via plug: wrapper ---
	snd_pcm_close(pcm_handle);
	pcm_handle = nullptr;
	{
		// Build "plug:SLAVE='<original>'" — ALSA's plug plugin converts
		// channels, format, and rate transparently.
		char plug_name[512];
		snprintf(plug_name, sizeof(plug_name), "plug:'%s'", dev_name);
		fprintf(stderr, "ALSA: device '%s' requires %u channels; "
			"trying '%s' for automatic conversion\n",
			dev_name, n_channels, plug_name);
		if (try_open(plug_name) == 0) {
			rc = try_configure(plug_name);
			if (rc == 0) goto configured;
			snd_pcm_close(pcm_handle);
			pcm_handle = nullptr;
		}
	}

	// --- Attempt 3: try plughw directly (bypasses user's asoundrc) ---
	fprintf(stderr, "ALSA: plug wrapper failed; trying plughw:0,0\n");
	if (try_open("plughw:0,0") == 0) {
		rc = try_configure("plughw:0,0");
		if (rc == 0) goto configured;
		snd_pcm_close(pcm_handle);
		pcm_handle = nullptr;
	}

	// --- Attempt 4: fall back to original device, accept any channel count ---
	fprintf(stderr, "ALSA: falling back to '%s' with native channel count\n", dev_name);
	if (try_open(dev_name) < 0) {
		fprintf(stderr, "ALSA: cannot reopen PCM '%s': %s\n", dev_name, snd_strerror(err));
		return 1;
	}
	{
		snd_pcm_hw_params_t *params;
		snd_pcm_hw_params_alloca(&params);
		snd_pcm_hw_params_any(pcm_handle, params);

		snd_pcm_hw_params_set_access(pcm_handle, params,
			SND_PCM_ACCESS_RW_INTERLEAVED);

		use_float = true;
		if (snd_pcm_hw_params_set_format(pcm_handle, params,
					SND_PCM_FORMAT_FLOAT) < 0) {
			use_float = false;
			snd_pcm_hw_params_set_format(pcm_handle, params,
				SND_PCM_FORMAT_S16_LE);
		}

		n_channels = 2;
		snd_pcm_hw_params_set_channels_near(pcm_handle, params, &n_channels);

		unsigned int rate = 48000;
		snd_pcm_hw_params_set_rate_near(pcm_handle, params, &rate, nullptr);
		synth->setSampleRate(rate);

		snd_pcm_uframes_t period = BufSize;
		snd_pcm_hw_params_set_period_size_near(pcm_handle, params, &period, nullptr);
		snd_pcm_uframes_t buffer_size = period * 4;
		snd_pcm_hw_params_set_buffer_size_near(pcm_handle, params, &buffer_size);

		if ((err = snd_pcm_hw_params(pcm_handle, params)) < 0) {
			fprintf(stderr, "ALSA: cannot set PCM hw params (fallback): %s\n", snd_strerror(err));
			return 1;
		}
		snd_pcm_prepare(pcm_handle);

		snd_pcm_uframes_t actual_period = 0;
		snd_pcm_hw_params_get_period_size(params, &actual_period, nullptr);
		period_frames = (int)actual_period;

		fprintf(stderr, "ALSA PCM: device='%s' (fallback) rate=%u format=%s channels=%u "
			"period=%d\n",
			dev_name, rate, use_float ? "FLOAT" : "S16_LE", n_channels, period_frames);
	}

configured:
	// Allocate mono accumulation buffer and interleave write buffers.
	mono_accum.assign((size_t)period_frames + (size_t)BufSize, 0.0f);
	accum_fill = 0;
	pcm_buf_f.assign((size_t)period_frames * n_channels, 0.0f);
	pcm_buf_s.assign((size_t)period_frames * n_channels, 0);
	return 0;
}

// Write nframes mono float samples to ALSA PCM, duplicated across all channels.
// Retries on partial writes (blocking mode can still return short).
void AlsaDriver::writePCM(float *buf, int nframes) {
	const void  *data;
	size_t frame_bytes;

	// Duplicate mono to first 2 channels (front L+R) only.
	// Extra surround channels stay zeroed (buffers are zero-initialized)
	// to prevent surround-sound DSP from garbling the audio.
	const unsigned int out_ch = std::min(n_channels, 2u);

	if (use_float) {
		frame_bytes = sizeof(float) * n_channels;
		if (n_channels == 1) {
			data = buf;
		} else {
			for (int i = 0; i < nframes; i++)
				for (unsigned int ch = 0; ch < out_ch; ch++)
					pcm_buf_f[(size_t)i * n_channels + ch] = buf[i];
			data = pcm_buf_f.data();
		}
	} else {
		frame_bytes = sizeof(int16_t) * n_channels;
		for (int i = 0; i < nframes; i++) {
			float s = buf[i];
			if (s >  1.0f) s =  1.0f;
			if (s < -1.0f) s = -1.0f;
			int16_t v = (int16_t)(s * 32767.0f);
			for (unsigned int ch = 0; ch < out_ch; ch++)
				pcm_buf_s[(size_t)i * n_channels + ch] = v;
		}
		data = pcm_buf_s.data();
	}

	// Retry loop handles partial writes (rare but valid in blocking mode)
	int frames_left = nframes;
	const uint8_t *ptr = static_cast<const uint8_t *>(data);
	while (frames_left > 0) {
		snd_pcm_sframes_t rc = snd_pcm_writei(pcm_handle, ptr,
		                                       (snd_pcm_uframes_t)frames_left);
		if (rc < 0) {
			// Recover from underrun or other errors
			rc = snd_pcm_recover(pcm_handle, rc, 1);
			if (rc < 0) {
				fprintf(stderr, "ALSA PCM write error (fatal): %s\n", snd_strerror(rc));
				// Prepare PCM after fatal error
				snd_pcm_prepare(pcm_handle);
				break;
			}
		} else if (rc > 0) {
			frames_left -= (int)rc;
			ptr         += (size_t)rc * frame_bytes;
		}
	}
}

// Enumerate ALSA seq clients and connect our port to matching hardware ports.
// in_rx / out_rx are substring filters:
//   nullptr or "" → don't connect in/out at all
//   "auto"        → connect to all hardware ports (skipping Midi Through)
//   other string  → only connect to ports whose name contains that substring
void AlsaDriver::autoConnectMidi(const char *in_rx, const char *out_rx) {
	bool do_in  = (in_rx  && *in_rx);
	bool do_out = (out_rx && *out_rx);
	if (!do_in && !do_out) return;

	// Helper: return true if the port name matches the pattern.
	// "auto" = match any hardware port (but never Midi Through).
	auto nameMatch = [](const char *pattern, const char *portname) -> bool {
		// Always skip Midi Through — it echoes everything back and causes loops
		if (strstr(portname, "Midi Through") || strstr(portname, "MIDI Through"))
			return false;
		if (strcmp(pattern, "auto") == 0) return true; // all hardware ports
		return strstr(portname, pattern) != nullptr;
	};

	snd_seq_client_info_t *cinfo;
	snd_seq_port_info_t   *pinfo;
	snd_seq_client_info_alloca(&cinfo);
	snd_seq_port_info_alloca(&pinfo);

	snd_seq_client_info_set_client(cinfo, -1);
	while (snd_seq_query_next_client(seq, cinfo) >= 0) {
		int client = snd_seq_client_info_get_client(cinfo);
		if (client == our_client)            continue; // skip ourselves
		if (client == SND_SEQ_CLIENT_SYSTEM) continue; // skip ALSA system client

		snd_seq_port_info_set_client(pinfo, client);
		snd_seq_port_info_set_port(pinfo, -1);
		while (snd_seq_query_next_port(seq, pinfo) >= 0) {
			unsigned int caps     = snd_seq_port_info_get_capability(pinfo);
			int          port     = snd_seq_port_info_get_port(pinfo);
			const char  *portname = snd_seq_port_info_get_name(pinfo);

			// Connect input: ports that can produce MIDI (READ + SUBS_READ)
			if (do_in &&
				(caps & (SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ)) ==
				        (SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ) &&
				nameMatch(in_rx, portname)) {
				if (snd_seq_connect_from(seq, seq_port, client, port) == 0)
					fprintf(stderr, "ALSA MIDI: input from %d:%d (%s)\n",
					        client, port, portname);
			}

			// Connect output: ports that can receive MIDI (WRITE + SUBS_WRITE)
			if (do_out &&
				(caps & (SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE)) ==
				        (SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE) &&
				nameMatch(out_rx, portname)) {
				if (snd_seq_connect_to(seq, seq_port, client, port) == 0)
					fprintf(stderr, "ALSA MIDI: output to %d:%d (%s)\n",
					        client, port, portname);
			}
		}
	}
}

// Main audio + MIDI loop (runs in its own thread)
void AlsaDriver::audioLoop() {
	// Try to get real-time priority for the audio thread
	// This improves latency and reduces underruns
	struct sched_param param;
	param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 10;
	if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
		// RT priority failed (common without JACK or special privileges),
		// but the thread will still work, just with higher latency jitter
		param.sched_priority = sched_get_priority_max(SCHED_RR) - 10;
		sched_setscheduler(0, SCHED_RR, &param); // try round-robin as fallback
	}

	while (running) {
		// --- MIDI input: drain all pending seq events ---
		if (seq) {
			snd_seq_event_t *ev;
			while (snd_seq_event_input_pending(seq, 1) > 0) {
				if (snd_seq_event_input(seq, &ev) < 0) break;

				uint8_t buf[4];
				int size = 0;

				switch (ev->type) {
				case SND_SEQ_EVENT_NOTEON:
					buf[0] = 0x90 | (ev->data.note.channel & 0x0F);
					buf[1] = ev->data.note.note;
					buf[2] = ev->data.note.velocity;
					size = 3;
					break;
				case SND_SEQ_EVENT_NOTEOFF:
					buf[0] = 0x80 | (ev->data.note.channel & 0x0F);
					buf[1] = ev->data.note.note;
					buf[2] = ev->data.note.velocity;
					size = 3;
					break;
				case SND_SEQ_EVENT_KEYPRESS:
					buf[0] = 0xA0 | (ev->data.note.channel & 0x0F);
					buf[1] = ev->data.note.note;
					buf[2] = ev->data.note.velocity;
					size = 3;
					break;
				case SND_SEQ_EVENT_CONTROLLER:
					buf[0] = 0xB0 | (ev->data.control.channel & 0x0F);
					buf[1] = (uint8_t)ev->data.control.param;
					buf[2] = (uint8_t)ev->data.control.value;
					size = 3;
					break;
				case SND_SEQ_EVENT_PGMCHANGE:
					buf[0] = 0xC0 | (ev->data.control.channel & 0x0F);
					buf[1] = (uint8_t)ev->data.control.value;
					size = 2;
					break;
				case SND_SEQ_EVENT_CHANPRESS:
					buf[0] = 0xD0 | (ev->data.control.channel & 0x0F);
					buf[1] = (uint8_t)ev->data.control.value;
					size = 2;
					break;
				case SND_SEQ_EVENT_PITCHBEND: {
					int pb = ev->data.control.value + 8192;
					buf[0] = 0xE0 | (ev->data.control.channel & 0x0F);
					buf[1] = pb & 0x7F;
					buf[2] = (pb >> 7) & 0x7F;
					size = 3;
					break;
				}
				case SND_SEQ_EVENT_SYSEX:
					synth->queueMidiRx(ev->data.ext.len,
						(const uint8_t *)ev->data.ext.ptr);
					break;
				default:
					break;
				}
				if (size > 0) synth->queueMidiRx(size, buf);
			}
		}

		// --- Generate audio (always BufSize frames) ---
		synth->run();

		// Capture audio to WAV file if requested
		capWrite(synth->outputBuffer, BufSize);

		// Append to mono accumulation buffer.
		// mono_accum is sized period_frames + BufSize, so there is always room.
		memcpy(mono_accum.data() + accum_fill, synth->outputBuffer,
		       (size_t)BufSize * sizeof(float));
		accum_fill += BufSize;

		// --- MIDI output: forward synth TX events to sequencer ---
		if (seq && midi_encoder) {
			uint32_t size = 0;
			uint8_t *buf  = nullptr;
			while (synth->queueMidiTx(size, buf)) {
				for (uint32_t i = 0; i < size; i++) {
					snd_seq_event_t ev;
					snd_seq_ev_clear(&ev);
					long rc = snd_midi_event_encode_byte(midi_encoder, buf[i], &ev);
					if (rc == 1) {
						snd_seq_ev_set_source(&ev, seq_port);
						snd_seq_ev_set_subs(&ev);
						snd_seq_ev_set_direct(&ev);
						snd_seq_event_output_direct(seq, &ev);
					}
				}
			}
		}

		// --- Write complete periods to PCM once we have enough frames.
		//     Loop so that when period_frames < BufSize we drain all ready
		//     periods in one go, keeping accum_fill < period_frames.
		//     snd_pcm_writei() blocks when the ring buffer is full, which
		//     naturally paces the loop to real time. ---
		while (accum_fill >= period_frames) {
			writePCM(mono_accum.data(), period_frames);
			int remainder = accum_fill - period_frames;
			if (remainder > 0)
				memmove(mono_accum.data(),
				        mono_accum.data() + period_frames,
				        (size_t)remainder * sizeof(float));
			accum_fill = remainder;
		}
	}
}

// Called from main(): open PCM, open seq, connect MIDI
void AlsaDriver::init() {
	if (openPCM(pcm_dev)) {
		fprintf(stderr, "Could not open ALSA PCM device.\n");
		exit(1);
	}
	if (openSeq()) {
		fprintf(stderr, "Could not open ALSA sequencer.\n");
		exit(1);
	}
}

// Start auto-connecting MIDI and launch audio thread
int AlsaDriver::initMidi(const char *in_rx, const char *out_rx) {
	autoConnectMidi(in_rx, out_rx);

	running = true;
	audio_thread = std::thread(&AlsaDriver::audioLoop, this);
	return 0;
}

// Set PCM device name (must be called before init())
int AlsaDriver::initPCM(const char *dev_name) {
	pcm_dev = dev_name;
	return 0;
}
