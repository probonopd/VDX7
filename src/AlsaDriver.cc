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

#include "AlsaDriver.h"

AlsaDriver::~AlsaDriver() {
	running = false;
	if (audio_thread.joinable()) audio_thread.join();
	if (midi_encoder) { snd_midi_event_free(midi_encoder); midi_encoder = nullptr; }
	if (seq) { snd_seq_close(seq); seq = nullptr; }
	if (pcm_handle) { snd_pcm_drain(pcm_handle); snd_pcm_close(pcm_handle); pcm_handle = nullptr; }
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

	if ((err = snd_pcm_open(&pcm_handle, dev_name,
				SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
		fprintf(stderr, "ALSA: cannot open PCM '%s': %s\n", dev_name, snd_strerror(err));
		return 1;
	}

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

	// Prefer stereo; fall back to whatever channel count the device supports
	n_channels = 2;
	if ((err = snd_pcm_hw_params_set_channels_near(pcm_handle, params, &n_channels)) < 0) {
		fprintf(stderr, "ALSA: cannot set channel count: %s\n", snd_strerror(err));
		return 1;
	}
	if (n_channels != 1 && n_channels != 2) {
		fprintf(stderr, "ALSA: device supports neither mono nor stereo (%u ch)\n", n_channels);
		return 1;
	}

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

	fprintf(stderr, "ALSA PCM: device='%s' rate=%u format=%s channels=%u\n",
		dev_name, rate, use_float ? "FLOAT" : "S16_LE", n_channels);
	return 0;
}

// Write BufSize mono float samples as stereo (or mono) to ALSA PCM
void AlsaDriver::writePCM(float *buf, int nframes) {
	// Use fixed-size arrays based on the compile-time constant Synth::BufSize.
	// nframes is always BufSize (128), so the stereo buffer is at most 256 elements.
	static float   pcm_f[Synth::BufSize * 2];
	static int16_t pcm_s[Synth::BufSize * 2];

	snd_pcm_sframes_t rc;
	if (use_float) {
		if (n_channels == 2) {
			for (int i = 0; i < nframes; i++) {
				pcm_f[i*2]   = buf[i];
				pcm_f[i*2+1] = buf[i];
			}
			rc = snd_pcm_writei(pcm_handle, pcm_f, nframes);
		} else {
			rc = snd_pcm_writei(pcm_handle, buf, nframes);
		}
	} else {
		if (n_channels == 2) {
			for (int i = 0; i < nframes; i++) {
				float s = buf[i];
				if (s >  1.0f) s =  1.0f;
				if (s < -1.0f) s = -1.0f;
				int16_t v = (int16_t)(s * 32767.0f);
				pcm_s[i*2]   = v;
				pcm_s[i*2+1] = v;
			}
			rc = snd_pcm_writei(pcm_handle, pcm_s, nframes);
		} else {
			for (int i = 0; i < nframes; i++) {
				float s = buf[i];
				if (s >  1.0f) s =  1.0f;
				if (s < -1.0f) s = -1.0f;
				pcm_s[i] = (int16_t)(s * 32767.0f);
			}
			rc = snd_pcm_writei(pcm_handle, pcm_s, nframes);
		}
	}

	if (rc < 0) {
		rc = snd_pcm_recover(pcm_handle, rc, 0);
		if (rc < 0)
			fprintf(stderr, "ALSA PCM write error: %s\n", snd_strerror(rc));
	}
}

// Enumerate ALSA seq clients and connect our port to matching hardware ports
void AlsaDriver::autoConnectMidi(const char *in_rx, const char *out_rx) {
	bool do_in  = (in_rx  && *in_rx);
	bool do_out = (out_rx && *out_rx);
	if (!do_in && !do_out) return;

	snd_seq_client_info_t *cinfo;
	snd_seq_port_info_t   *pinfo;
	snd_seq_client_info_alloca(&cinfo);
	snd_seq_port_info_alloca(&pinfo);

	snd_seq_client_info_set_client(cinfo, -1);
	while (snd_seq_query_next_client(seq, cinfo) >= 0) {
		int client = snd_seq_client_info_get_client(cinfo);
		if (client == our_client)         continue; // skip ourselves
		if (client == SND_SEQ_CLIENT_SYSTEM) continue; // skip ALSA system client

		snd_seq_port_info_set_client(pinfo, client);
		snd_seq_port_info_set_port(pinfo, -1);
		while (snd_seq_query_next_port(seq, pinfo) >= 0) {
			unsigned int caps = snd_seq_port_info_get_capability(pinfo);
			int port          = snd_seq_port_info_get_port(pinfo);

			// Connect input: ports that can produce MIDI (READ + SUBS_READ)
			if (do_in &&
				(caps & (SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ)) ==
				        (SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ)) {
				if (snd_seq_connect_from(seq, seq_port, client, port) == 0)
					fprintf(stderr, "ALSA MIDI: connected input from %d:%d\n", client, port);
			}

			// Connect output: ports that can receive MIDI (WRITE + SUBS_WRITE)
			if (do_out &&
				(caps & (SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE)) ==
				        (SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE)) {
				if (snd_seq_connect_to(seq, seq_port, client, port) == 0)
					fprintf(stderr, "ALSA MIDI: connected output to %d:%d\n", client, port);
			}
		}
	}
}

// Main audio + MIDI loop (runs in its own thread)
void AlsaDriver::audioLoop() {
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

		// --- Generate audio ---
		synth->run();

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

		// --- Write audio to PCM (blocks until hardware needs more data) ---
		writePCM(synth->outputBuffer, BufSize);
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
