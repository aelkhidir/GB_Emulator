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

class Memory;
class CPU;

enum class Mode
{
	Hblank = 0u,
	Vblank = 1u,
	OAMSearch = 2u,
	PixelTransfer = 3u
};

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

class PPU
{
public:
	PPU(Memory* memory, CPU* cpu) : memory(memory), cpu(cpu) {}

	Memory* memory;
	CPU* cpu;

	// Screen
	SDL_Event event;
	SDL_Renderer* renderer;
	SDL_Window* window;

	// Registers
	uint8_t LCDControl;
	uint8_t stat;
	uint8_t SCY;
	uint8_t SCX;
	uint8_t LY;
	uint8_t LYC;
	uint8_t BGPalette;
	uint8_t OBP0, OBP1;
	

	uint64_t clock = 0;

	void Init();
	std::array<uint8_t, 256 * 256> DecodeTileMap();
	std::array<uint8_t, 256 * 256> DecodeOAM(std::array<uint8_t, 256 * 256>& colorChoices);
	std::array<uint8_t, 64> ExtractTileData(uint8_t offset);
	std::array<uint8_t, 64> ExtractTileDataUnsigned(uint8_t offset);
	uint8_t GetColor(uint8_t index);
	uint8_t GetObjectColor(uint8_t index, bool paletteChoice);
	void DrawScreen();
	void RenderTileMap();
	Mode CalculatePPUMode(uint64_t offset = 0);
	void DrawFullBackground();
	void DrawTileMap();
	std::map<std::pair<uint8_t, uint8_t>, uint8_t> ExtractFullTileMap();
	bool CheckPPUtoVRAMAccess();
	void Terminate();
	void Update(uint64_t cycleCount);


};