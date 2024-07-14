#pragma once
#define SDL_MAIN_HANDLED
#include<cassert>
#include<iostream>
#include<sstream>
#include<fstream>
#include<format>
#include<vector>
#include<string>
#include<array>
#include<map>
#include<queue>
#include<Windows.h>
#include<SDL.h>
#include<cstdio>
#include<cstdint>

class Memory;
class CPU;
class PPU;
class APU;
class Joypad;

class GameBoy
{
public:
	GameBoy();
	~GameBoy();
	
private:
	Memory* memory;
	CPU* cpu;
	PPU* ppu;
	APU* apu;
	Joypad* joypad;

	// real clock for emulating FPS
	clock_t realTime = clock();

public:
	void Reset();
	void SetLogging(bool printCPULogs);
	void SetToPostBootState();
	void ExecutionLoop();
	void WaitForFrameTime();
	bool HandleInterrupts();
	void PointToCartridgeMemory();
	void StartDMATransfer(uint16_t address);
	bool CanAccessVRAM();
	bool CanAccessOAM();
	void UpdateClock(uint32_t cycleCount);
	void LoadRom(std::string filename, uint16_t startAddress);
	void LoadCartridge(std::string filename);
};
