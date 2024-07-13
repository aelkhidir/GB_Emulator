#include "apu.hpp"
#include "memory.hpp"
#include "helpers.hpp"

void APU::InitAPU()
{
	SDL_OpenAudio(&AudioSettings, 0);
	SDL_Init(SDL_INIT_AUDIO);
	AudioSettings.freq = 44100;
	AudioSettings.format = AUDIO_F32SYS;
	AudioSettings.channels = 1;
	AudioSettings.samples = 1024;
	SDL_OpenAudioDevice(SDL_GetAudioDeviceName(1, 0), 0, &AudioSettings, &AudioSettingsObtained, 0);
	//audio_state << std::string(SDL_GetAudioDeviceName(1, 0)) << std::endl;
}

void APU::IncrementDivAPU()
{
	div_apu_clock++;
	SweepChannel1();
	UpdateChannel1Envelope();
	UpdateChannel2Envelope();
}

void APU::Update(uint64_t cycles)
{
	SwitchChannel1();
	SwitchChannel2();
	SwitchChannel3();
	ControlChannel3();

	ch1_period_divider += cycles / 4;
	ch2_period_divider += cycles / 4;
	ch3_period_divider += cycles / 2;

	if (ch1_period_divider > 0x07FF)
	{
		ch1_period_divider = ((uint16_t(ch1_period_high) & 0b111) << 8) + ch1_period_low;
		//// duty cycle (fraction out of 8)
		//uint8_t wave_0[8] = { 0, 0, 0, 0, 0, 0, 0, 1 };
		//uint8_t wave_1[8] = { 1, 0, 0, 0, 0, 0, 0, 1 };
		//uint8_t wave_2[8] = { 1, 0, 0, 0, 0, 1, 1, 1 };
		//uint8_t wave_3[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };
		//switch ((ch1_timer & 0b11000000) >> 6)
		//{
		//case 0:
		//{
		//	ch1_buffer[ch1_sample] = wave_0[ch1_sample % 8] * ch1_volume;
		//	break;
		//}
		//case 1:
		//{
		//	ch1_buffer[ch1_sample] = wave_1[ch1_sample % 8] * ch1_volume;
		//	break;
		//}
		//case 2:
		//{
		//	ch1_buffer[ch1_sample] = wave_2[ch1_sample % 8] * ch1_volume;
		//	break;
		//}
		//case 3:
		//{
		//	ch1_buffer[ch1_sample] = wave_3[ch1_sample % 8] * ch1_volume;
		//	break;
		//}
		//default:
		//	_ASSERTE(false);
		//}

		//if (ch1_sample != audio_buffer_size - 1)
		//{
		//	ch1_sample = (ch1_sample + 1) % audio_buffer_size;
		//}
	}

	if (ch2_period_divider > 0x07FF)
	{
		ch2_period_divider = ((uint16_t(ch2_period_high) & 0b111) << 8) + ch2_period_low;
		//// duty cycle (fraction out of 8)
		//uint8_t wave_0[8] = { 0, 0, 0, 0, 0, 0, 0, 1 };
		//uint8_t wave_1[8] = { 1, 0, 0, 0, 0, 0, 0, 1 };
		//uint8_t wave_2[8] = { 1, 0, 0, 0, 0, 1, 1, 1 };
		//uint8_t wave_3[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };
		//switch ((ch2_timer & 0b11000000) >> 6)
		//{
		//case 0:
		//{
		//	ch2_buffer[ch2_sample] = wave_0[ch2_sample % 8] * ch2_volume;
		//	break;
		//}
		//case 1:
		//{
		//	ch2_buffer[ch2_sample] = wave_1[ch2_sample % 8] * ch2_volume;
		//	break;
		//}
		//case 2:
		//{
		//	ch2_buffer[ch2_sample] = wave_2[ch2_sample % 8] * ch2_volume;
		//	break;
		//}
		//case 3:
		//{
		//	ch2_buffer[ch2_sample] = wave_3[ch2_sample % 8] * ch2_volume;
		//	break;
		//}
		//default:
		//	_ASSERTE(false);
		//}

		//if (ch2_sample != audio_buffer_size - 1)
		//{
		//	ch2_sample = (ch2_sample + 1) % audio_buffer_size;
		//}
	}


	GetChannel1Buffer();
	GetChannel2Buffer();

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

	double delta = (clock() - audio_clock) / double(CLOCKS_PER_SEC);
	double start = audio_clock / CLOCKS_PER_SEC;
	if (delta > 0.01)
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

		//std::vector<uint8_t> ch1_samples;
		//std::vector<uint8_t> ch2_samples;
		//std::vector<uint8_t> ch3_samples;
		std::vector<float> samples;
		uint32_t sample_delta = AudioSettings.freq * delta;
		uint32_t start_sample = AudioSettings.freq * start;
		for (uint32_t i = 0; i < sample_delta; i++) {
			// SDL_QueueAudio expects a signed 16-bit value
			// note: "5000" here is just gain so that we will hear something
			//uint8_t sample = (sin(x * 4) + 1) * 128;
			//float ch1_sample = ch1_buffer[(uint16_t(i * ch1_sample_stride)) % audio_buffer_size];
			//float ch2_sample = ch2_buffer[(uint16_t(i * ch2_sample_stride)) % audio_buffer_size];
			float ch1_sample = ch1_volume * sin(float(i) * 2.0f * M_PI * ch1_frequency / double(AudioSettings.freq));
			float ch2_sample = ch2_volume * sin(float(i) * 2.0f * M_PI * ch2_frequency / double(AudioSettings.freq));
			float ch3_sample = ch3_buffer[(uint16_t(i * ch3_sample_stride)) % audio_buffer_size];
			float sample = 0;

			// TODO: FIX AUDIO

			/*if (Helpers::ExtractBit(control, 0))
			{
				sample += ch1_sample * 512;
			}*/

			/*if (Helpers::ExtractBit(control, 1))
			{
				sample += ch2_sample * 512;
			}*/

			/*if (Helpers::ExtractBit(memory->Get(0xFF1A), 7))
			{
				sample += ch3_sample * 128;
			}*/

			//ch1_samples.push_back(ch1_sample);
			//ch2_samples.push_back(ch2_sample);
			//ch3_samples.push_back(ch3_sample);
			//uint16_t sample = 4000 + 4000 * sin(double(i) * 2.0f * M_PI * 400 / double(AudioSettings.freq));
			samples.push_back(0);
			
		}
		//std::vector<uint8_t> mixed_samples;
		//mixed_samples.resize(ch1_samples.size());
		//SDL_MixAudioFormat(mixed_samples.data(), ch1_samples.data(), AUDIO_U8, ch1_samples.size(), 8);
		//SDL_MixAudioFormat(mixed_samples.data(), ch2_samples.data(), AUDIO_U8, ch2_samples.size(), 8);
		//SDL_MixAudioFormat(mixed_samples.data(), ch3_samples.data(), AUDIO_U8, ch3_samples.size(), 8);
		/*for (auto sample : samples)
		{
			audio_state << sample << "\n";
		}*/

		//audio_state << "Buffer size: " << samples.size() << std::endl;

		if (samples.size() == 0)
		{
			__debugbreak();
		}

		if (Helpers::ExtractBit(control, 7))
		{
			int success = SDL_QueueAudio(1, samples.data(), samples.size() * 4);
			//audio_state << "Success: " << success << std::endl;
		}
		SDL_PauseAudioDevice(1, 0);
		audio_clock = clock();
	}
	/*clock_t new_clock = clock();
	float delta = float(new_clock - audio_clock) / float(CLOCKS_PER_SEC);

	for (uint32_t i = 0; i < audio_buffer_size; i++)
	{
		output_buffer[i] = ch1_buffer[i] / 64 + ch2_buffer[i] / 64;
	}

	if (Helpers::ExtractBit(control, 7))
	{
		SDL_QueueAudio(2, output_buffer, unsigned(AudioSettings.freq * delta));
	}
	SDL_PauseAudioDevice(2, 0);
	audio_clock = clock();*/
}

void APU::SwitchChannel1()
{
	if (!Helpers::ExtractBit(ch1_period_high, 7))
	{
		return;
	}

	ch1_length_clock = ch1_timer & 0b00111111;
	ch1_volume = (ch1_envelope & 0xf0) >> 4;
	control |= 1;
}

void APU::SwitchChannel2()
{
	if (!Helpers::ExtractBit(ch2_period_high, 7))
	{
		return;
	}
	ch2_length_clock = ch2_timer & 0b00111111;
	ch2_volume = (ch2_envelope & 0xf0) >> 4;
	control |= (1 << 1);
}

void APU::SwitchChannel3()
{
	if (!Helpers::ExtractBit(ch3_period_high, 7))
	{
		return;
	}

	ch1_length_clock = ch3_timer;
	control |= (1 << 2);
}

void APU::ControlChannel3()
{
	if (!Helpers::ExtractBit(ch3_dac_enable, 7)) { control &= ~(1 << 2); }
	else { control |= (1 << 2); }
}

void APU::SweepChannel1()
{
	uint8_t ch1_pace = (ch1_sweep >> 4) & 0b111;
	uint8_t ch1_ticks = ch1_pace * 4;

	if (ch1_ticks == 0)
	{
		return;
	}

	if (div_apu_clock % ch1_ticks != 0)
	{
		return;
	}

	uint8_t step = ch1_sweep & 0x07;
	bool direction = (ch1_sweep >> 3) & 0b111;
	uint8_t two_pow_step = (1 << step);
	uint16_t period = ch1_period_divider;
	uint16_t new_period = direction ? period - (period / two_pow_step) : period + (period / two_pow_step);
	if (new_period > 0x07FF)
	{
		// turn off
		control &= ~(1 << 0);
		ch1_period_divider = (uint16_t(ch1_period_high & 0b111) << 8) + ch1_period_low;
		return;
	}
	ch1_period_divider = new_period;
}

void APU::UpdateChannel1Envelope()
{
	uint8_t sweep_pace = ch1_envelope & 0b111;
	uint8_t sweep_ticks = 8 * sweep_pace;

	if (sweep_ticks == 0)
	{
		return;
	}

	if (div_apu_clock % sweep_ticks != 0)
	{
		return;
	}

	bool direction = (ch1_envelope >> 3) & 0x01;
	if (!direction && ch1_volume == 0)
	{
		return;
	}

	ch1_volume = direction ? ch1_volume + 1 : ch1_volume - 1;
}

void APU::UpdateChannel2Envelope()
{
	uint8_t sweep_pace = ch2_envelope & 0b111;
	uint8_t sweep_ticks = 8 * sweep_pace;

	if (sweep_ticks == 0)
	{
		return;
	}

	if (div_apu_clock % sweep_ticks != 0)
	{
		return;
	}

	bool direction = (ch2_envelope >> 3) & 0x01;
	if (!direction && ch2_volume == 0)
	{
		return;
	}

	ch2_volume = direction ? ch2_volume + 1 : ch2_volume - 1;
}


void APU::GetChannel1Buffer()
{
	uint8_t duty_cycle = (ch1_length_clock >> 6) & 0b11;
	uint8_t wave_0[8] = { 0, 0, 0, 0, 0, 0, 0, 1 };
	uint8_t wave_1[8] = { 1, 0, 0, 0, 0, 0, 0, 1 };
	uint8_t wave_2[8] = { 1, 0, 0, 0, 0, 1, 1, 1 };
	uint8_t wave_3[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };
	uint32_t samples_per_second = PeriodToFrequency(ch1_period_divider);
	uint32_t steps_per_sample = AudioSettings.freq / samples_per_second;
	switch (duty_cycle)
	{
	case 0:
	{
		for (uint32_t i = 0; i < audio_buffer_size; i++)
		{
			uint32_t index = (i * steps_per_sample) % 8;
			ch1_buffer[i] = wave_0[index] * ch1_volume;
		}
		break;
	}
	case 1:
	{
		for (uint32_t i = 0; i < audio_buffer_size; i++)
		{
			uint32_t index = (i * steps_per_sample) % 8;
			ch1_buffer[i] = wave_1[index] * ch1_volume;
		}
		break;
	}
	case 2:
	{
		for (uint32_t i = 0; i < audio_buffer_size; i++)
		{
			uint32_t index = (i * steps_per_sample) % 8;
			ch1_buffer[i] = wave_2[index] * ch1_volume;
		}
		break;
	}
	case 3:
	{
		for (uint32_t i = 0; i < audio_buffer_size; i++)
		{
			uint32_t index = (i * steps_per_sample) % 8;
			ch1_buffer[i] = wave_3[index] * ch1_volume;
		}
		break;
	}
	}
}

void APU::GetChannel2Buffer()
{
	uint8_t duty_cycle = (ch2_length_clock >> 6) & 0b11;
	uint8_t wave_0[8] = { 0, 0, 0, 0, 0, 0, 0, 1 };
	uint8_t wave_1[8] = { 1, 0, 0, 0, 0, 0, 0, 1 };
	uint8_t wave_2[8] = { 1, 0, 0, 0, 0, 1, 1, 1 };
	uint8_t wave_3[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };
	uint32_t samples_per_second = PeriodToFrequency(ch2_period_divider);
	uint32_t steps_per_sample = AudioSettings.freq / samples_per_second;
	switch (duty_cycle)
	{
	case 0:
	{
		for (uint32_t i = 0; i < audio_buffer_size; i++)
		{
			uint32_t index = (i * steps_per_sample) % 8;
			ch2_buffer[i] = wave_0[index] * ch2_volume;
		}
		break;
	}
	case 1:
	{
		for (uint32_t i = 0; i < audio_buffer_size; i++)
		{
			uint32_t index = (i * steps_per_sample) % 8;
			ch2_buffer[i] = wave_1[index] * ch2_volume;
		}
		break;
	}
	case 2:
	{
		for (uint32_t i = 0; i < audio_buffer_size; i++)
		{
			uint32_t index = (i * steps_per_sample) % 8;
			ch2_buffer[i] = wave_2[index] * ch2_volume;
		}
		break;
	}
	case 3:
	{
		for (uint32_t i = 0; i < audio_buffer_size; i++)
		{
			uint32_t index = (i * steps_per_sample) % 8;
			ch2_buffer[i] = wave_3[index] * ch2_volume;
		}
		break;
	}
	}
}


uint32_t APU::PeriodToFrequency(uint16_t period)
{
	_ASSERT(period < 0x800);
	uint32_t dots_per_sample = 0x800 - period;
	uint32_t dots_per_second = 4194304;
	uint32_t samples_per_second = dots_per_second / dots_per_sample;
	return samples_per_second;
}

