#include "gameboy.hpp"


int main()
{
	bool skipBootRom = true;

	GameBoy gameboy;
	gameboy.Reset();
	gameboy.LoadRom("bootrom.gb", 0x0000);
	gameboy.LoadCartridge("tests/09-op r,r.gb");

	if (skipBootRom)
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