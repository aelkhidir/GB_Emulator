#include "gameboy.hpp"


int main()
{
	bool postBootState = true;

	GameBoy gameboy;
	gameboy.LoadRom("bootrom.gb", 0x0000);
	gameboy.LoadCartridge("tetris.gb");

	if (postBootState)
	{
		gameboy.SetToPostBootState();
	}

	else
	{
		gameboy.Reset();
	}

	gameboy.ExecutionLoop();
    return EXIT_SUCCESS;
}