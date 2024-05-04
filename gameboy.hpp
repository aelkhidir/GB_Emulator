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
#include"memory.hpp"
#include"joypad.hpp"
#include"apu.hpp"
#include"cpu.hpp"

enum class Mode;

struct Pixel
{
	Pixel(uint8_t color = 0, uint8_t palette = 0, uint8_t spritePriority = 0, uint8_t backgroundPriority = 0)
	{
		this->color = color;
		this->palette = palette;
		this->spritePriority = spritePriority;
		this->backgroundPriority = backgroundPriority;
	}

	uint8_t color;
	uint8_t palette;
	uint8_t spritePriority;
	uint8_t backgroundPriority;
};



class GameBoy
{
public:
	GameBoy() : 
		memory(&cpu, &apu, &joypad),
		apu(&memory),
		cpu(&memory, &apu),
		joypad(&memory, &cpu)
	{}
	
private:
	class FIFO
	{
	private:
		std::queue<Pixel> pixelQueue;
		GameBoy& gameboy;
		uint8_t fetcherX;
		uint8_t fetcherY;

	public:
		FIFO(GameBoy& gameboy) : gameboy(gameboy), fetcherX(0), fetcherY(0) {}
		void FetchPixels();
		uint8_t CalculateTileOffset();
		std::array<uint8_t, 64> GetFetcherTileData();
	};


	uint64_t ppu_cycles = 0;

	Memory memory;
	CPU cpu;
	APU apu;
	Joypad joypad;

	// real clock for emulating FPS
	clock_t realTime = clock();

	// Screen
	SDL_Event event;
	SDL_Renderer* renderer;
	SDL_Window* window;

	// Pixel FIFO
	FIFO backgroundFifo = FIFO(*this);
	FIFO spriteFifo = FIFO(*this);


public:
	void Reset();
	void SetToPostBootState();
	void ExecutionLoop();
	void WaitForFrameTime();
	bool HandleInterrupts();
	void PointToCartridgeMemory();
	void Write(uint8_t value, uint16_t address);
	uint8_t Read(uint16_t address);
	void StartDMATransfer(uint16_t address);
	void WriteScanlineY(uint8_t value);
	void UpdatePPU(uint64_t cycleCount);
	bool CanAccessVRAM();
	bool CanAccessOAM();
	std::array<uint8_t, 256 * 256> DecodeTileMap();
	std::array<uint8_t, 256 * 256> DecodeOAM();
	std::array<uint8_t, 64> ExtractTileData(uint8_t offset);
	std::array<uint8_t, 64> ExtractTileDataUnsigned(uint8_t offset);
	uint8_t GetColor(uint8_t index);
	void DrawScreen();
	void RenderTileMap();
	Mode CalculatePPUMode(uint64_t offset=0);
	void UpdateClock(uint32_t cycleCount);
	void LoadRom(std::string filename, uint16_t startAddress);
	void LoadCartridge(std::string filename);
	void DrawFullBackground();
	void DrawTileMap();
	std::map<std::pair<uint8_t, uint8_t>, uint8_t> ExtractFullTileMap();
	bool CheckPPUtoVRAMAccess();
};


enum class Mode
{
	Hblank = 0u,
	Vblank = 1u,
	OAMSearch = 2u,
	PixelTransfer = 3u
};