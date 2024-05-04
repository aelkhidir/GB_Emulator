#include "apu.hpp"
#include "memory.hpp"
#include "helpers.hpp"

void APU::InitAPU()
{
	SDL_OpenAudio(&AudioSettings, 0);
	SDL_Init(SDL_INIT_AUDIO);
	AudioSettings.freq = 44100;
	AudioSettings.format = AUDIO_U16;
	AudioSettings.channels = 1;
	AudioSettings.samples = 1024;
	SDL_OpenAudioDevice(SDL_GetAudioDeviceName(2, 0), 0, &AudioSettings, &AudioSettingsObtained, 0);
}

// Todo: remove direct access to the memory pointer and use a bus component instead
void APU::ReadState()
{
	control = memory->Get(0xFF26);
	panning = memory->Get(0xFF25);
	volume_and_vin_panning = memory->Get(0xFF24);

	ch1_sweep = memory->Get(0xFF10);
	ch1_timer = memory->Get(0xFF11);
	ch1_envelope = memory->Get(0xFF12);
	ch1_period_low = memory->Get(0xFF13);
	ch1_period_high = memory->Get(0xFF14);

	ch2_timer = memory->Get(0xFF16);
	ch2_envelope = memory->Get(0xFF17);
	ch2_period_low = memory->Get(0xFF18);
	ch2_period_high = memory->Get(0xFF19);

	ch3_dac_enable = memory->Get(0xFF1A);
	ch3_timer = memory->Get(0xFF1B);
	ch3_output_level = memory->Get(0xFF1C);
	ch3_period_low = memory->Get(0xFF1D);
	ch3_period_high = memory->Get(0xFF1E);

	ch4_timer = memory->Get(0xFF20);
	ch4_envelope = memory->Get(0xFF21);
	ch4_frequency = memory->Get(0xFF22);
	ch4_control = memory->Get(0xFF23);
}

void APU::WriteState()
{
	memory->Set(0xFF26, control);
	memory->Set(0xFF25, panning);
	memory->Set(0xFF24, volume_and_vin_panning);
	memory->Set(0xFF10, ch1_sweep);
	memory->Set(0xFF11, ch1_timer);
	memory->Set(0xFF12, ch1_envelope);
	memory->Set(0xFF13, ch1_period_low);
	memory->Set(0xFF14, ch1_period_high);
	memory->Set(0xFF16, ch2_timer);
	memory->Set(0xFF17, ch2_envelope);
	memory->Set(0xFF18, ch2_period_low);
	memory->Set(0xFF19, ch2_period_high);
	memory->Set(0xFF1A, ch3_dac_enable);
	memory->Set(0xFF1B, ch3_timer);
	memory->Set(0xFF1C, ch3_output_level);
	memory->Set(0xFF1D, ch3_period_low);
	memory->Set(0xFF1E, ch3_period_high);
	memory->Set(0xFF20, ch4_timer);
	memory->Set(0xFF21, ch4_envelope);
	memory->Set(0xFF22, ch4_frequency);
	memory->Set(0xFF23, ch4_control);
}

void APU::StepAPU()
{
	div_apu_clock++;
	ReadState();

	// If off, clear all registers and do nothing
	if (!Helpers::ExtractBit(control, 7))
	{
		for (uint16_t i = 0xFF10; i < 0xFF24; i++)
		{
			if (i == 0xFF15 || i == 0xFF1F)
			{
				continue;
			}
			memory->Set(i, 0);
		}
		return;
	}

	// Channel 1 sound
	if (Helpers::ExtractBit(control, 0))
	{
		uint8_t ch1_pace = (ch1_sweep & (0b01110000)) >> 4;
		uint8_t ch1_direction = Helpers::ExtractBit(ch1_sweep, 3);
		if (Helpers::ExtractBit(ch1_period_high, 6))
		{
			ch1_length_clock++;
			if (ch1_length_clock >= 64)
			{
				control &= ~1;
			}
		}

		// Period sweep
		if (ch1_pace != 0 && div_apu_clock % (4 * ch1_pace) == 0)
		{
			uint8_t ch1_step = (ch1_sweep & 0b00000111);
			uint16_t ch1_period = ch1_period_low + ((uint16_t(ch1_period_high) & 0b00000111) << 8);
			uint16_t ch1_period_change = ch1_period / (ldexp(1, ch1_step));
			ch1_period = ch1_direction ? ch1_period - ch1_period_change : ch1_period + ch1_period_change;
			if (ch1_period > 0x7FF)
			{
				control &= ~1;
			}
			else
			{
				ch1_period_low = (ch1_period & 0xff);
				ch1_period_high = (ch1_period & 0b11100000000) >> 8;
			}
		}

		uint8_t ch1_envelope_pace = ch1_envelope & 0b111;

		if (ch1_envelope_pace != 0 && div_apu_clock % (8 * ch1_envelope_pace) == 0)
		{
			if (Helpers::ExtractBit(ch1_envelope, 3)) { ch1_volume = std::min<uint8_t>(15, ch1_volume + 1); }
			else { ch1_volume = std::max<uint8_t>(0, ch1_volume - 1); }

		}

	}

	// Channel 2
	if (Helpers::ExtractBit(control, 1))
	{
		if (Helpers::ExtractBit(ch2_period_high, 6))
		{
			ch2_length_clock++;
			if (ch2_length_clock >= 64)
			{
				control &= ~1;
			}
		}

		uint8_t ch2_envelope_pace = ch2_envelope & 0b111;

		if (ch2_envelope_pace != 0 && div_apu_clock % (8 * ch2_envelope_pace) == 0)
		{
			if (Helpers::ExtractBit(ch2_envelope, 3)) { ch2_volume = std::min<uint8_t>(15, ch2_volume + 1); }
			else { ch2_volume = std::max<uint8_t>(0, ch2_volume - 1); }

		}
	}

	// Channel 3
	if (Helpers::ExtractBit(control, 2))
	{
		if (Helpers::ExtractBit(ch3_period_high, 6))
		{
			ch3_length_clock++;
			if (ch3_length_clock >= 64)
			{
				control &= ~(1 << 2);
			}
		}
	}

	// Channel 4
	if (Helpers::ExtractBit(control, 3))
	{

	}

	WriteState();
}

void APU::UpdateAPUTimers(uint64_t cycles)
{
	ReadState();

	ch1_period_divider += cycles;
	ch2_period_divider += cycles;
	ch3_period_divider += 2 * cycles;

	if (ch1_period_divider > 0x07FF)
	{
		ch1_period_divider = ((uint16_t(ch1_period_high) & 0b111) << 8) + ch1_period_low;
		// duty cycle (fraction out of 8)
		uint8_t wave_0[8] = { 0, 0, 0, 0, 0, 0, 0, 1 };
		uint8_t wave_1[8] = { 1, 0, 0, 0, 0, 0, 0, 1 };
		uint8_t wave_2[8] = { 1, 0, 0, 0, 0, 1, 1, 1 };
		uint8_t wave_3[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };
		switch ((ch1_timer & 0b11000000) >> 6)
		{
		case 0:
		{
			ch1_buffer[ch1_sample] = wave_0[ch1_sample % 8] * ch1_volume;
			break;
		}
		case 1:
		{
			ch1_buffer[ch1_sample] = wave_1[ch1_sample % 8] * ch1_volume;
			break;
		}
		case 2:
		{
			ch1_buffer[ch1_sample] = wave_2[ch1_sample % 8] * ch1_volume;
			break;
		}
		case 3:
		{
			ch1_buffer[ch1_sample] = wave_3[ch1_sample % 8] * ch1_volume;
			break;
		}
		default:
			_ASSERTE(false);
		}

		if (ch1_sample != audio_buffer_size - 1)
		{
			ch1_sample = (ch1_sample + 1) % audio_buffer_size;
		}
	}

	if (ch2_period_divider > 0x07FF)
	{
		ch2_period_divider = ((uint16_t(ch2_period_high) & 0b111) << 8) + ch2_period_low;
		// duty cycle (fraction out of 8)
		uint8_t wave_0[8] = { 0, 0, 0, 0, 0, 0, 0, 1 };
		uint8_t wave_1[8] = { 1, 0, 0, 0, 0, 0, 0, 1 };
		uint8_t wave_2[8] = { 1, 0, 0, 0, 0, 1, 1, 1 };
		uint8_t wave_3[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };
		switch ((ch2_timer & 0b11000000) >> 6)
		{
		case 0:
		{
			ch2_buffer[ch2_sample] = wave_0[ch2_sample % 8] * ch2_volume;
			break;
		}
		case 1:
		{
			ch2_buffer[ch2_sample] = wave_1[ch2_sample % 8] * ch2_volume;
			break;
		}
		case 2:
		{
			ch2_buffer[ch2_sample] = wave_2[ch2_sample % 8] * ch2_volume;
			break;
		}
		case 3:
		{
			ch2_buffer[ch2_sample] = wave_3[ch2_sample % 8] * ch2_volume;
			break;
		}
		default:
			_ASSERTE(false);
		}

		if (ch2_sample != audio_buffer_size - 1)
		{
			ch2_sample = (ch2_sample + 1) % audio_buffer_size;
		}
	}

	if (ch3_period_divider > 0x07FF)
	{
		uint16_t ch3_period_value = ((uint16_t(ch3_period_high) & 0b111) << 8) + ch3_period_low;
		ch3_period_divider = ch3_period_value;
		uint8_t volume_array[4] = { 0, 16, 8, 4 };
		uint8_t volume = volume_array[((ch3_output_level & 0b01100000) >> 5)];
		uint8_t sample_byte = memory->Get(0xFF30 + (ch3_sample % 0x20) / 2);
		ch3_buffer[ch3_sample] = ch3_sample % 2 ? (sample_byte & 0x0f) : (sample_byte >> 4);
		ch3_buffer[ch3_sample] *= volume;
		ch3_sample = (ch3_sample + 1) % audio_buffer_size;
	}


	if (((ch3_sample % 128 == 0)))
	{
		uint16_t ch1_period_value = ((uint16_t(ch1_period_high) & 0b111) << 8) + ch1_period_low;
		uint16_t ch2_period_value = ((uint16_t(ch2_period_high) & 0b111) << 8) + ch2_period_low;
		uint16_t ch3_period_value = ((uint16_t(ch3_period_high) & 0b111) << 8) + ch3_period_low;

		float ch1_sample_rate = 1048576 / (2048 - ch1_period_value);
		float ch2_sample_rate = 1048576 / (2048 - ch2_period_value);
		float ch3_sample_rate = 2097152 / (2048 - ch3_period_value);

		float ch1_sample_stride = ch1_sample_rate / float(AudioSettings.freq);
		float ch2_sample_stride = ch2_sample_rate / float(AudioSettings.freq);
		float ch3_sample_stride = ch3_sample_rate / float(AudioSettings.freq);

		float ch1_frequency = ch1_sample_rate / 8;
		float ch2_frequency = ch2_sample_rate / 8;

		clock_t new_clock = clock();
		float delta = float(new_clock - audio_clock) / float(CLOCKS_PER_SEC);
		//std::vector<uint8_t> ch1_samples;
		//std::vector<uint8_t> ch2_samples;
		//std::vector<uint8_t> ch3_samples;
		std::vector<uint16_t> samples;
		for (uint32_t i = 0; i < AudioSettings.freq * delta; i++) {
			// SDL_QueueAudio expects a signed 16-bit value
			// note: "5000" here is just gain so that we will hear something
			//uint8_t sample = (sin(x * 4) + 1) * 128;
			//uint8_t ch1_sample = ch1_buffer[(uint16_t(i * ch1_sample_stride)) % audio_buffer_size];
			//uint8_t ch2_sample = ch2_buffer[(uint16_t(i * ch2_sample_stride)) % audio_buffer_size];

			uint16_t ch1_sample = ch1_volume * sin(float(i) * 2.0f * M_PI * ch1_frequency / float(AudioSettings.freq));
			uint16_t ch2_sample = ch2_volume * sin(float(i) * 2.0f * M_PI * ch2_frequency / float(AudioSettings.freq));
			uint16_t ch3_sample = ch3_buffer[(uint16_t(i * ch3_sample_stride)) % audio_buffer_size];
			uint16_t sample = 0;

			/*if (Helpers::ExtractBit(control, 0))
			{
				sample += ch1_sample / 32;
			}

			if (Helpers::ExtractBit(control, 1))
			{
				sample += ch2_sample / 32;
			}*/

			if (Helpers::ExtractBit(memory->Get(0xFF1A), 7))
			{
				sample += ch3_sample * 16;
			}

			//ch1_samples.push_back(ch1_sample);
			//ch2_samples.push_back(ch2_sample);
			//ch3_samples.push_back(ch3_sample);
			samples.push_back(sample);
		}
		//std::vector<uint8_t> mixed_samples;
		//mixed_samples.resize(ch1_samples.size());
		//SDL_MixAudioFormat(mixed_samples.data(), ch1_samples.data(), AUDIO_U8, ch1_samples.size(), 8);
		//SDL_MixAudioFormat(mixed_samples.data(), ch2_samples.data(), AUDIO_U8, ch2_samples.size(), 8);
		//SDL_MixAudioFormat(mixed_samples.data(), ch3_samples.data(), AUDIO_U8, ch3_samples.size(), 8);
		if (Helpers::ExtractBit(control, 7))
		{
			SDL_QueueAudio(2, samples.data(), samples.size() * 2);
		}
		SDL_PauseAudioDevice(2, 0);
		audio_clock = clock();
	}

	WriteState();
}

void APU::TriggerChannel1()
{
	ch1_timer = memory->Get(0xFF11);
	ch1_envelope = memory->Get(0xFF12);
	control = memory->Get(0xFF26);
	ch1_length_clock = ch1_timer & 0b00111111;
	ch1_volume = (ch1_envelope & 0xf0) >> 4;
	control |= 1;
}

void APU::TriggerChannel2()
{
	ch2_timer = memory->Get(0xFF16);
	ch2_envelope = memory->Get(0xFF17);
	control = memory->Get(0xFF26);
	ch2_length_clock = ch2_timer & 0b00111111;
	ch2_volume = (ch2_envelope & 0xf0) >> 4;
	control |= (1 << 1);
}

void APU::TriggerChannel3()
{
	ch3_timer = memory->Get(0xFF1B);
	control = memory->Get(0xFF26);
	ch1_length_clock = ch3_timer;
	control |= (1 << 2);
}

void APU::ControlChannel3(uint8_t value)
{
	control = memory->Get(0xFF26);
	if (!Helpers::ExtractBit(value, 7)) { control &= ~(1 << 2); }
	else { control |= (1 << 2); }
}

