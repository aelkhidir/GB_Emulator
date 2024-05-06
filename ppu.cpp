#include "ppu.hpp"
#include "memory.hpp"
#include "cpu.hpp"
#include "helpers.hpp"

void PPU::Init()
{
	SDL_Init(SDL_INIT_VIDEO);
	SDL_CreateWindowAndRenderer(800, 720, 0, &window, &renderer);
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);
	SDL_PumpEvents();
	clock = 0;
}

std::array<uint8_t, 256 * 256> PPU::DecodeTileMap()
{
	uint16_t startAddress = Helpers::ExtractBit(LCDControl, 3) ? 0x9C00 : 0x9800;
	std::array<uint8_t, 256 * 256> backgroundColors;
	for (uint32_t tileNum = 0; tileNum < 32 * 32; tileNum++)
	{
		std::array<uint8_t, 64> tileData = ExtractTileData(memory->mainMemory[startAddress + tileNum]);
		uint32_t baseX = (tileNum % 32) * 8;
		uint32_t baseY = (tileNum / 32) * 8;

		for (uint32_t pixelNum = 0; pixelNum < 64; pixelNum++)
		{
			uint32_t offsetX = pixelNum % 8;
			uint32_t offsetY = pixelNum / 8;
			backgroundColors[(baseX + offsetX) + 256 * (baseY + offsetY)] = GetColor(tileData[pixelNum]);
		}
	}
	return backgroundColors;
}


std::array<uint8_t, 256 * 256> PPU::DecodeOAM()
{
	struct sprite
	{
		uint8_t Y;
		uint8_t X;
		uint8_t tile_index;
		uint8_t flags;
	};

	std::vector<sprite> sprite_vec;
	for (uint16_t address = 0xFE00; address < 0xFE9F; address += 4)
	{
		sprite_vec.emplace_back(memory->mainMemory[address], memory->mainMemory[address + 1], memory->mainMemory[address + 2], memory->mainMemory[address + 3]);
	}

	std::array<uint8_t, 256 * 256> sprite_colors;

	// Unused space is -1
	// todo: std optional?
	for (auto& element : sprite_colors)
	{
		element = -1;
	}

	for (auto const& object : sprite_vec)
	{
		// Support 8x8 and 8x16 modes
		std::array<uint8_t, 64> tileData = ExtractTileDataUnsigned(object.tile_index);
		uint32_t baseX = SCX + object.X;
		uint32_t baseY = SCY + object.Y;
		for (uint32_t pixelNum = 0; pixelNum < 64; pixelNum++)
		{
			uint32_t offsetX = pixelNum % 8;
			uint32_t offsetY = pixelNum / 8;
			if (offsetY + object.Y >= 16 && offsetX + object.X >= 8)
			{
				sprite_colors[(baseX + offsetX - 8) + 256 * (baseY + offsetY - 16)] = GetColor(tileData[pixelNum]);
			}
		}
	}
	return sprite_colors;
}

std::map<std::pair<uint8_t, uint8_t>, uint8_t> PPU::ExtractFullTileMap()
{
	std::map<std::pair<uint8_t, uint8_t>, uint8_t> backgroundColors;
	for (uint32_t tileNum = 0; tileNum < 0x400; tileNum++)
	{
		std::array<uint8_t, 64> tileData = ExtractTileData(tileNum);
		uint32_t baseX = (tileNum % 0x20) * 8;
		uint32_t baseY = (tileNum / 0x20) * 8;

		for (uint32_t pixelNum = 0; pixelNum < 64; pixelNum++)
		{
			uint32_t offsetX = pixelNum % 8;
			uint32_t offsetY = pixelNum / 8;
			backgroundColors[{baseX + offsetX, baseY + offsetY}] = GetColor(tileData[pixelNum]);
		}
	}
	return backgroundColors;
}

void PPU::DrawScreen()
{
	std::array<uint8_t, 256 * 256> backgroundColors = DecodeTileMap();

	SDL_RenderSetLogicalSize(renderer, 160, 144);
	SDL_RenderClear(renderer);

	// Background/Window
	for (uint8_t offsetX = 0; offsetX < 160; offsetX++)
	{
		for (uint8_t offsetY = 0; offsetY < 144; offsetY++)
		{
			uint8_t color = 255 - 85 * backgroundColors[((SCX + offsetX) % 256) + 256 * ((SCY + offsetY) % 256)];
			SDL_SetRenderDrawColor(renderer, color, color, color, 255);
			SDL_RenderDrawPoint(renderer, offsetX, offsetY);
		}
	}

	// Objects
	std::array<uint8_t, 256 * 256> objectColors = DecodeOAM();
	for (uint8_t offsetX = 0; offsetX < 160; offsetX++)
	{
		for (uint8_t offsetY = 0; offsetY < 144; offsetY++)
		{
			uint8_t color_index = objectColors[((SCX + offsetX) % 256) + 256 * ((SCY + offsetY) % 256)];
			if (color_index == 0xff)
			{
				continue;
			}
			uint8_t color = 255 - 85 * color_index;
			SDL_SetRenderDrawColor(renderer, color, color, color, 255);
			SDL_RenderDrawPoint(renderer, offsetX, offsetY);
		}
	}

	SDL_RenderPresent(renderer);
	SDL_PumpEvents();
}

void PPU::DrawFullBackground()
{
	std::array<uint8_t, 256 * 256> backgroundColors = DecodeTileMap();
	SDL_RenderSetLogicalSize(renderer, 256, 256);
	SDL_RenderClear(renderer);
	for (uint16_t offsetX = 0; offsetX < 256; offsetX++)
	{
		for (uint16_t offsetY = 0; offsetY < 256; offsetY++)
		{
			uint8_t color = 255 - 85 * backgroundColors[offsetX + 256 * offsetY];
			SDL_SetRenderDrawColor(renderer, color, color, color, 255);
			SDL_RenderDrawPoint(renderer, offsetX, offsetY);
		}
	}
	SDL_RenderPresent(renderer);
}

void PPU::DrawTileMap()
{
	{
		std::map<std::pair<uint8_t, uint8_t>, uint8_t> backgroundColors = ExtractFullTileMap();
		SDL_RenderSetLogicalSize(renderer, 256, 256);
		SDL_RenderClear(renderer);
		for (uint16_t offsetX = 0; offsetX < 256; offsetX++)
		{
			for (uint16_t offsetY = 0; offsetY < 256; offsetY++)
			{
				uint8_t color = 255 - 85 * backgroundColors[{offsetX, offsetY}];
				SDL_SetRenderDrawColor(renderer, color, color, color, 255);
				SDL_RenderDrawPoint(renderer, offsetX, offsetY);
			}
		}
		SDL_RenderPresent(renderer);
	}
}

void PPU::RenderTileMap()
{
	std::vector<uint8_t> backgroundMap;
	std::vector<uint8_t> tileMap;
	std::vector<uint8_t> logo;

	for (uint16_t i = 0x9800; i < 0x9C00; i++)
	{
		backgroundMap.push_back(memory->mainMemory[i]);
	}

	for (uint16_t i = 0x8000; i < 0x8800; i++)
	{
		tileMap.push_back(memory->mainMemory[i]);
	}

	for (uint16_t i = 0x104; i < 0x134; i++)
	{
		logo.push_back(memory->mainMemory[i]);
	}

	return;
}

std::array<uint8_t, 64> PPU::ExtractTileData(uint8_t offset)
{
	int8_t signed_offset = offset;
	bool mapSelect = Helpers::ExtractBit(LCDControl, 4);
	uint16_t tileAddress = mapSelect ? 0x8000 + 16 * offset : 0x9000 + signed_offset * 16;
	std::array<uint8_t, 64> colorSelect;

	for (uint8_t pixelNum = 0; pixelNum < 64; pixelNum++)
	{
		uint8_t bit = 7 - (pixelNum % 8);
		uint8_t rowNum = pixelNum / 8;
		uint8_t paletteChoice = uint8_t(Helpers::ExtractBit(memory->mainMemory[tileAddress + 2 * rowNum], bit))
			+ uint8_t(Helpers::ExtractBit(memory->mainMemory[tileAddress + 2 * rowNum + 1], bit) << 1);
		colorSelect[pixelNum] = paletteChoice;
	}

	return colorSelect;
}

std::array<uint8_t, 64> PPU::ExtractTileDataUnsigned(uint8_t offset)
{
	uint16_t tileAddress = 0x8000 + 16 * offset;
	std::array<uint8_t, 64> colorSelect;

	for (uint8_t pixelNum = 0; pixelNum < 64; pixelNum++)
	{
		uint8_t bit = 7 - (pixelNum % 8);
		uint8_t rowNum = pixelNum / 8;
		uint8_t paletteChoice = uint8_t(Helpers::ExtractBit(memory->mainMemory[tileAddress + 2 * rowNum], bit))
			+ uint8_t(Helpers::ExtractBit(memory->mainMemory[tileAddress + 2 * rowNum + 1], bit) << 1);
		colorSelect[pixelNum] = paletteChoice;
	}

	return colorSelect;
}


uint8_t PPU::GetColor(uint8_t index)
{
	_ASSERTE(index < 4);

	switch (index)
	{
	case 0:
		return (BGPalette & 0b00000011);
	case 1:
		return (BGPalette & 0b00001100) >> 2;
	case 2:
		return (BGPalette & 0b00110000) >> 4;
	case 3:
		return (BGPalette & 0b11000000) >> 6;
	default:
		return 0;
	}
}

Mode PPU::CalculatePPUMode(uint64_t offset)
{
	// mode 2 -> mode 3 -> mode 0
	uint32_t scanlineClock = (clock + offset) % 456;
	uint8_t previousScanlineNumber = clock / 456;
	uint8_t nextScanlineNumber = (clock + offset) / 456;
	LY += (nextScanlineNumber - previousScanlineNumber);

	if (LY >= 144)
	{
		return Mode::Vblank;
	}

	else if (scanlineClock < 80)
	{
		return Mode::OAMSearch;
	}

	// todo: add sprite-dependent time to mode 3
	else if (scanlineClock < 80 + 172)
	{
		return Mode::PixelTransfer;
	}

	else
	{
		return Mode::Hblank;
	}
}

void PPU::Update(uint64_t cycleCount)
{
	uint64_t previousScanlineNumber = clock / 456;
	uint64_t nextScanlineNumber = (clock + cycleCount) / 456;

	Mode previousMode = Mode(stat & 0b11);
	Mode nextMode = CalculatePPUMode(cycleCount);

	if (!Helpers::ExtractBit(LCDControl, 7))
	{
		LY = 0;
		clock = 0;
		stat &= 0b11111100;
		return;
	}


	stat &= 0b11111100;
	stat |= uint8_t(nextMode);


	if (previousMode != nextMode)
	{
		if (nextMode == Mode::Vblank)
		{
			cpu->RequestVblankInterrupt();
		}
		if (nextMode == Mode::Hblank && Helpers::ExtractBit(stat, 3) || nextMode == Mode::Vblank && Helpers::ExtractBit(stat, 4)
			|| nextMode == Mode::OAMSearch && Helpers::ExtractBit(stat, 5) || nextMode == Mode::PixelTransfer && Helpers::ExtractBit(stat, 6))
		{
			cpu->RequestStatInterrupt();
		}
	}

	if (nextScanlineNumber != previousScanlineNumber)
	{
		LY = nextScanlineNumber;

		if (LY == LYC)
		{
			stat |= (1 << 2);
			if (Helpers::ExtractBit(stat, 6))
			{
				cpu->RequestStatInterrupt();
			}
		}

		else
		{
			stat &= ~(1 << 2);
		}
	}

	clock += cycleCount;
}

bool PPU::CheckPPUtoVRAMAccess()
{
	bool LCDOn = Helpers::ExtractBit(LCDControl, 7);
	bool scanlineZero = LY == 0;

	// todo: when searching OAM and index 37 is reached

	Mode currentMode = CalculatePPUMode();
	bool modeAllowsAccess = currentMode == Mode::PixelTransfer;

	return LCDOn && (scanlineZero || modeAllowsAccess);
}

void PPU::Terminate()
{
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}