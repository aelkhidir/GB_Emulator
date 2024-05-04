#include "gameboy.hpp"
#include "helpers.hpp"

void GameBoy::Reset()
{
	SDL_Init(SDL_INIT_VIDEO);
	SDL_CreateWindowAndRenderer(800, 720, 0, &window, &renderer);
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);
	SDL_PumpEvents();

	apu.InitAPU();
}

void GameBoy::SetToPostBootState()
{
	Reset();
	cpu.Init();
	memory.SetToPostBootState();
	ppu_cycles = 0;
}

void GameBoy::WriteScanlineY(uint8_t value)
{
	memory.mainMemory[0xFF44] = value;
}

void GameBoy::UpdatePPU(uint64_t cycleCount)
{ 
	uint64_t previousScanlineNumber = ppu_cycles / 456;
	uint64_t nextScanlineNumber = (ppu_cycles + cycleCount) / 456;

	uint8_t& stat = memory.mainMemory[0xFF41];
	Mode previousMode = Mode(stat & 0b11);
	Mode nextMode = CalculatePPUMode(cycleCount);

	uint8_t& control = memory.mainMemory[0xFF40];

	if (!Helpers::ExtractBit(control, 7))
	{
		WriteScanlineY(0);
		ppu_cycles = 0;
		stat &= 0b11111100;
		return;
	}

	
	uint8_t& interruptRequests = memory.mainMemory[0xFF0F];

	stat &= 0b11111100;
	stat |= uint8_t(nextMode);


	if (previousMode != nextMode)
	{
		if (nextMode == Mode::Vblank)
		{
			interruptRequests |= (1 << 0);
		}
		if (nextMode == Mode::Hblank && Helpers::ExtractBit(stat, 3) || nextMode == Mode::Vblank && Helpers::ExtractBit(stat, 4)
			|| nextMode == Mode::OAMSearch && Helpers::ExtractBit(stat, 5) || nextMode == Mode::PixelTransfer && Helpers::ExtractBit(stat, 6))
		{
			interruptRequests |= (1 << 1);
		}
	}

	if (nextScanlineNumber != previousScanlineNumber)
	{
		uint8_t LY = memory.mainMemory[0xFF44];
		uint8_t LYC = memory.mainMemory[0xFF45];
		WriteScanlineY(nextScanlineNumber);

		if (LY == LYC)
		{
			stat |= (1 << 2);
			if (Helpers::ExtractBit(stat, 6))
			{
				interruptRequests |= (1 << 1);
			}
		}

		else
		{
			stat &= ~(1 << 2);
		}
	}

	ppu_cycles += cycleCount;
}

void GameBoy::ExecutionLoop()
{
	while (true)
	{
		/*if (cpu_cycles == 494400) { __debugbreak(); }*/
		if (!cpu.haltMode && cpu.printLogs)
		{
			cpu.LogCPUStateDetailed();
		}

		if (cpu.HandleInterrupts())
		{
			cpu.LogCPUStateDetailed();
		}

		uint16_t previousPC = cpu.programCounter;
		if (!cpu.haltMode)
		{
			uint8_t opcode = cpu.ReadROMLine();
			cpu.ExecuteInstruction(opcode);
		}
		else
		{
			UpdateClock(4);
		}
	

		if (ppu_cycles >= 70224)
		{
			if (Helpers::ExtractBit(memory.mainMemory[0xFF40], 7) && Helpers::ExtractBit(memory.mainMemory[0xFF40], 0))
			{
				DrawScreen();
				WaitForFrameTime();
				ppu_cycles %= 70224;
			}		
		}
	}

	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}

void GameBoy::WaitForFrameTime()
{
	uint64_t millisecondsPassed = std::clock() - realTime;
	realTime = std::clock();

	if (millisecondsPassed > 17) 
	{ 
		return; 
	}

	else
	{
		Sleep(17 - millisecondsPassed);
		realTime = std::clock();
	}
}


void GameBoy::LoadRom(std::string filename, uint16_t startAddress)
{
	std::ifstream file(filename, std::ifstream::binary);

	if (file.is_open())
	{
		std::vector<char> bytes(
			(std::istreambuf_iterator<char>(file)),
			(std::istreambuf_iterator<char>()));

		file.close();

		for (uint32_t byteNumber = 0; byteNumber < bytes.size(); byteNumber++)
		{
			memory.Set(startAddress + byteNumber, bytes[byteNumber]);
			/*memory.mainMemory[startAddress + byteNumber] = bytes[byteNumber];*/
		}
		return;
	}
	_ASSERTE(false);
}

void GameBoy::LoadCartridge(std::string filename)
{
	std::ifstream cart(filename, std::ifstream::binary);

	if (cart.is_open())
	{
		std::vector<char> bytes(
			(std::istreambuf_iterator<char>(cart)),
			(std::istreambuf_iterator<char>()));

		cart.close();

		for (uint32_t byteNumber = 0; byteNumber < bytes.size(); byteNumber++)
		{
			memory.cartridgeMemory[byteNumber] = bytes[byteNumber];
		}

		for (uint32_t byteNumber = 0x100; byteNumber < 0x8000; byteNumber++)
		{
			memory.mainMemory[byteNumber] = memory.cartridgeMemory[byteNumber];
		}

		memory.mbcState = static_cast<MBC>(memory.cartridgeMemory[0x147]);
		uint8_t memory_banks = bytes.size() / 0x4000;
		memory.bankBits = std::bit_width(memory_banks) - 1; // log2
		return;
	}
	_ASSERTE(false);
}


uint8_t GameBoy::GetColor(uint8_t index)
{
	uint8_t palette = memory.mainMemory[0xFF47];
	_ASSERTE(index < 4);

	switch (index)
	{
	case 0:
		return (palette & 0b00000011);
	case 1:
		return (palette & 0b00001100) >> 2;
	case 2:
		return (palette & 0b00110000) >> 4;
	case 3:
		return (palette & 0b11000000) >> 6;
	default:
		return 0;
	}
}

std::array<uint8_t, 256 * 256> GameBoy::DecodeTileMap()
{
	uint8_t LCDControl = memory.mainMemory[0xFF40];
	uint16_t startAddress = Helpers::ExtractBit(LCDControl, 3) ? 0x9C00 : 0x9800;
	std::array<uint8_t, 256 * 256> backgroundColors;
	for (uint32_t tileNum = 0; tileNum < 32 * 32; tileNum++)
	{
		std::array<uint8_t, 64> tileData = ExtractTileData(memory.mainMemory[startAddress + tileNum]);
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

std::array<uint8_t, 256 * 256> GameBoy::DecodeOAM()
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
		sprite_vec.emplace_back(memory.mainMemory[address], memory.mainMemory[address + 1], memory.mainMemory[address + 2], memory.mainMemory[address + 3]);
	}

	std::array<uint8_t, 256 * 256> sprite_colors;

	// Unused space is -1
	// todo: std optional?
	for (auto& element : sprite_colors)
	{
		element = -1;
	}

	uint8_t SCY = memory.mainMemory[0xFF42];
	uint8_t SCX = memory.mainMemory[0xFF43];

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

std::map<std::pair<uint8_t, uint8_t>, uint8_t> GameBoy::ExtractFullTileMap()
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

void GameBoy::DrawScreen()
{
	std::array<uint8_t, 256 * 256> backgroundColors = DecodeTileMap();
	uint8_t SCY = memory.mainMemory[0xFF42];
	uint8_t SCX = memory.mainMemory[0xFF43];

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

void GameBoy::DrawFullBackground()
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

void GameBoy::DrawTileMap()
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

void GameBoy::RenderTileMap()
{
	std::vector<uint8_t> backgroundMap;
	std::vector<uint8_t> tileMap;
	std::vector<uint8_t> logo;

	for (uint16_t i = 0x9800; i < 0x9C00; i++)
	{
		backgroundMap.push_back(memory.mainMemory[i]);
	}

	for (uint16_t i = 0x8000; i < 0x8800; i++)
	{
		tileMap.push_back(memory.mainMemory[i]);
	}

	for (uint16_t i = 0x104; i < 0x134; i++)
	{
		logo.push_back(memory.mainMemory[i]);
	}

	return;
}

std::array<uint8_t, 64> GameBoy::ExtractTileData(uint8_t offset)
{
	uint8_t LCDControl = memory.mainMemory[0xFF40];
	int8_t signed_offset = offset;
	bool mapSelect = Helpers::ExtractBit(LCDControl, 4);
	uint16_t tileAddress = mapSelect ?  0x8000 + 16 * offset : 0x9000 + signed_offset * 16;
	std::array<uint8_t, 64> colorSelect;

	for (uint8_t pixelNum = 0; pixelNum < 64; pixelNum++)
	{
		uint8_t bit = 7 - (pixelNum % 8);
		uint8_t rowNum = pixelNum / 8;
		uint8_t paletteChoice = uint8_t(Helpers::ExtractBit(memory.mainMemory[tileAddress + 2 * rowNum], bit)) 
			+ uint8_t(Helpers::ExtractBit(memory.mainMemory[tileAddress + 2 * rowNum + 1], bit) << 1);
		colorSelect[pixelNum] = paletteChoice;
	}

	return colorSelect;
}

std::array<uint8_t, 64> GameBoy::ExtractTileDataUnsigned(uint8_t offset)
{
	uint8_t LCDControl = memory.mainMemory[0xFF40];
	uint16_t tileAddress = 0x8000 + 16 * offset;
	std::array<uint8_t, 64> colorSelect;

	for (uint8_t pixelNum = 0; pixelNum < 64; pixelNum++)
	{
		uint8_t bit = 7 - (pixelNum % 8);
		uint8_t rowNum = pixelNum / 8;
		uint8_t paletteChoice = uint8_t(Helpers::ExtractBit(memory.mainMemory[tileAddress + 2 * rowNum], bit))
			+ uint8_t(Helpers::ExtractBit(memory.mainMemory[tileAddress + 2 * rowNum + 1], bit) << 1);
		colorSelect[pixelNum] = paletteChoice;
	}

	return colorSelect;
}

void GameBoy::FIFO::FetchPixels()
{
	// Get tile
	uint8_t LCDControl = gameboy.memory.mainMemory[0xFF40];
	uint8_t LY = gameboy.memory.mainMemory[0xFF44];
	uint16_t tileMapAddress = 0x9800;

	// todo: and scanline x in window
	if (Helpers::ExtractBit(LCDControl, 3))
	{
		tileMapAddress = 0x9C00;
	}

	// todo: and scanline x in window
	else if (Helpers::ExtractBit(LCDControl, 6))
	{
		tileMapAddress = 0x9C00;
	}

	bool isWindowTile = false;
	uint8_t X, Y;

	if (isWindowTile)
	{

	}

	

	// Get tile data low

	// Get tile data high

	// Sleep

	// Push
}

// if PPU can't access VRAM then just get 0xFF
std::array<uint8_t, 64> GameBoy::FIFO::GetFetcherTileData()
{
	if (gameboy.CheckPPUtoVRAMAccess())
	{

	}
	
	else
	{
		return gameboy.ExtractTileData(0xFF);
	}
}

uint8_t GameBoy::FIFO::CalculateTileOffset()
{
	// use fetcherX, fetcherY to calculate the offset
	uint8_t tileX = fetcherX / 8;
	uint8_t tileY = fetcherY / 8;

	return 32 * tileY + tileX;
}

bool GameBoy::CheckPPUtoVRAMAccess()
{
	uint8_t LCDControl = memory.mainMemory[0xFF40];
	uint8_t LY = memory.mainMemory[0xFF44];
	bool LCDOn = Helpers::ExtractBit(LCDControl, 7);
	bool scanlineZero = LY == 0;

	// todo: when searching OAM and index 37 is reached

	Mode currentMode = CalculatePPUMode();
	bool modeAllowsAccess = currentMode == Mode::PixelTransfer;

	return LCDOn && (scanlineZero || modeAllowsAccess);
}

// Calculates the modes needed for graphics operations (hblank, vblank etc)
Mode GameBoy::CalculatePPUMode(uint64_t offset)
{
	// mode 2 -> mode 3 -> mode 0
	uint32_t scanlineClock = (ppu_cycles + offset) % 456;
	uint8_t previousScanlineNumber = ppu_cycles / 456;
	uint8_t nextScanlineNumber = (ppu_cycles + offset) / 456;
	uint8_t LY = memory.mainMemory[0xFF44] + (nextScanlineNumber - previousScanlineNumber);

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

void GameBoy::UpdateClock(uint32_t cycleCount)
{
	UpdatePPU(cycleCount);
	apu.UpdateAPUTimers(cycleCount);
	cpu.UpdateClock(cycleCount);
}

