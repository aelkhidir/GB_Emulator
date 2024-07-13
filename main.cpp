#include "gameboy.hpp"


int main(int argc, char* argv[])
{
	bool postBootState = true;

	GameBoy gameboy;

	bool printCPULogs = false;
	if (argc > 1 && std::string(argv[1]) == "--cpu.printLogs")
	{
		printCPULogs = true;
	}
	gameboy.SetLogging(printCPULogs);
	
	gameboy.LoadRom("bootrom.gb", 0x0000);
	//gameboy.LoadCartridge("mts/emulator-only/mbc1/bits_bank1.gb");
	//gameboy.LoadCartridge("tests/01-special.gb");
	//gameboy.LoadCartridge("mts/emulator-only/mbc1/bits_bank1.gb");
	gameboy.LoadCartridge("dreamland.gb");
	//gameboy.LoadRom("tests/07-jr,jp,call,ret,rst.gb", 0x0000);
	//gameboy.LoadCartridge("tetris.gb");

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