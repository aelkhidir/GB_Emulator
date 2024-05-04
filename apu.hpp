#include<iostream>
#include<sstream>
#include<fstream>
#include<format>
#include<vector>
#include<string>
#include<array>
#include<map>
#include<queue>
#include<SDL.h>

constexpr auto audio_buffer_size = 4096;

class Memory;

class APU
{
private:
	// Audio
	uint16_t ch1_length_clock = 0;
	uint16_t ch2_length_clock = 0;
	uint16_t ch3_length_clock = 0;
	uint16_t ch1_period_divider = 0;
	uint16_t ch2_period_divider = 0;
	uint16_t ch3_period_divider = 0;

	uint8_t ch1_buffer[audio_buffer_size];
	uint8_t ch2_buffer[audio_buffer_size];
	uint8_t ch3_buffer[audio_buffer_size];
	uint16_t ch1_sample = 0;
	uint16_t ch2_sample = 0;
	uint16_t ch3_sample = 0;
	uint16_t ch3_sample_counter = 0;

	uint8_t ch1_volume = 0;
	uint8_t ch2_volume = 0;
	uint16_t div_apu_clock = 0;
	inline static Uint8* audio_position = 0;
	inline static Uint32 audio_length = 0;
	SDL_AudioSpec AudioSettings = { 0 };
	SDL_AudioSpec AudioSettingsObtained = { 0 };
	SDL_AudioDeviceID AudioDevice;
	clock_t audio_clock = clock();

	uint8_t control;
	uint8_t panning;
	uint8_t volume_and_vin_panning;

	uint8_t ch1_sweep;
	uint8_t ch1_timer;
	uint8_t ch1_envelope;
	uint8_t ch1_period_low;
	uint8_t ch1_period_high;

	uint8_t ch2_timer;
	uint8_t ch2_envelope;
	uint8_t ch2_period_low;
	uint8_t ch2_period_high;

	uint8_t ch3_dac_enable;
	uint8_t ch3_timer;
	uint8_t ch3_output_level;
	uint8_t ch3_period_low;
	uint8_t ch3_period_high;

	uint8_t ch4_timer;
	uint8_t ch4_envelope;
	uint8_t ch4_frequency;
	uint8_t ch4_control;
	Memory* memory;
public:
	APU(Memory* memory_pointer) : memory(memory_pointer) {}
	void InitAPU();
	void StepAPU();
	void UpdateAPUTimers(uint64_t cycles);
	void ReadState();
	void WriteState();
	void TriggerChannel1();
	void TriggerChannel2();
	void TriggerChannel3();
	void ControlChannel3(uint8_t value);
};