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

#pragma once

#include <alsa/asoundlib.h>
#include <thread>
#include <atomic>
#include <vector>
#include "Synth.h"

class AlsaDriver {
public:
	AlsaDriver(Synth *s) : synth(s), BufSize(Synth::BufSize) { }
	void init();
	int initMidi(const char *in_port_name, const char *out_port_name);
	int initPCM(const char *dev_name);
	virtual ~AlsaDriver();

private:
	int openPCM(const char *dev_name);
	int openSeq();
	void autoConnectMidi(const char *in_rx, const char *out_rx);
	void writePCM(float *buf, int nframes);
	void audioLoop();

	Synth *synth = nullptr;
	int BufSize;

	// ALSA PCM
	snd_pcm_t *pcm_handle = nullptr;
	bool use_float = true;
	unsigned int n_channels = 2; // actual channel count negotiated with ALSA
	std::vector<float>   pcm_buf_f; // interleave buffer for float writes
	std::vector<int16_t> pcm_buf_s; // interleave buffer for S16 writes

	// ALSA Sequencer (MIDI)
	snd_seq_t *seq = nullptr;
	int seq_port = -1;
	int our_client = -1;
	snd_midi_event_t *midi_encoder = nullptr;

	// Audio + MIDI thread
	std::thread audio_thread;
	std::atomic<bool> running{false};

	// Saved port connection options
	const char *midi_in_rx = nullptr;
	const char *midi_out_rx = nullptr;
	const char *pcm_dev = nullptr;
};
