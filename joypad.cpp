#include "joypad.hpp"
#include "memory.hpp"
#include "cpu.hpp"
#include "helpers.hpp"


uint8_t Joypad::GetJoypadInput()
{
	auto setBit = [&](uint8_t num, bool state) {
		joypad &= ~(1 << num);
		joypad |= (uint8_t(state) << num);
	};

	bool right_pressed = (GetKeyState(VK_RIGHT) & 0x8000);
	bool left_pressed = (GetKeyState(VK_LEFT) & 0x8000);
	bool up_pressed = (GetKeyState(VK_UP) & 0x8000);
	bool down_pressed = (GetKeyState(VK_DOWN) & 0x8000);
	bool start_pressed = (GetKeyState('A') & 0x8000);
	bool select_pressed = (GetKeyState('S') & 0x8000);
	bool a_pressed = (GetKeyState('D') & 0x8000);
	bool b_pressed = (GetKeyState('F') & 0x8000);

	if (!Helpers::ExtractBit(joypad, 4))
	{
		setBit(0, !right_pressed);
		setBit(1, !left_pressed);
		setBit(2, !up_pressed);
		setBit(3, !down_pressed);
	}

	else if (!Helpers::ExtractBit(joypad, 5))
	{
		setBit(0, !start_pressed);
		setBit(1, !select_pressed);
		setBit(2, !a_pressed);
		setBit(3, !b_pressed);
	}

	else
	{
		joypad |= 0x0F;
	}

	// Falling edges trigger the joypad interrupt
	if (Helpers::ExtractBit(previous_joypad, 0) && !Helpers::ExtractBit(joypad, 0))
	{
		cpu->RequestJoypadInterrupt();
	}

	else if (Helpers::ExtractBit(previous_joypad, 1) && !Helpers::ExtractBit(joypad, 1))
	{
		cpu->RequestJoypadInterrupt();
	}

	else if (Helpers::ExtractBit(previous_joypad, 2) && !Helpers::ExtractBit(joypad, 2))
	{
		cpu->RequestJoypadInterrupt();
	}

	else if (Helpers::ExtractBit(previous_joypad, 3) && !Helpers::ExtractBit(joypad, 3))
	{
		cpu->RequestJoypadInterrupt();
	}

	previous_joypad = joypad;
	return joypad;
}


void Joypad::SetJoypadValue(uint8_t value) 
{
	previous_joypad = joypad;
	joypad = value;
};

