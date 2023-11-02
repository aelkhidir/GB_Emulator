#include "gameboy.hpp"

void GameBoy::Reset()
{
	programCounter = 0x0000;
	stackPointer = 0x0000;
	SDL_Init(SDL_INIT_VIDEO);
	SDL_CreateWindowAndRenderer(800, 720, 0, &window, &renderer);
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);
	SDL_PumpEvents();
}

void GameBoy::SetToPostBootState()
{
	bool unmapBootRom = false;

	if (unmapBootRom)
	{
		UnmapBootRom();
	}

	// Set the states of each register correctly
	registerA = 0x01;
	registerF = 0xB0;
	registerB = 0x00;
	registerC = 0x13;
	registerD = 0x00;
	registerE = 0xD8;
	registerH = 0x01;
	registerL = 0x4D;
	programCounter = 0x0100;
	stackPointer = 0xFFFE;

	// hardware register values after the DMG boot rom
	memory[0xFF00] = 0xCF;
	memory[0xFF01] = 0x00;
	memory[0xFF02] = 0x7E;
	memory[0xFF04] = 0xAB;
	memory[0xFF05] = 0x00;
	memory[0xFF06] = 0x00;
	memory[0xFF07] = 0xF8;
	memory[0xFF0F] = 0xE1;
	memory[0xFF10] = 0x80;
	memory[0xFF11] = 0xBF;
	memory[0xFF12] = 0xF3;
	memory[0xFF13] = 0xFF;
	memory[0xFF14] = 0xBF;
	memory[0xFF16] = 0x3F;
	memory[0xFF17] = 0x00;
	memory[0xFF18] = 0xFF;
	memory[0xFF19] = 0xBF;
	memory[0xFF1A] = 0x7F;
	memory[0xFF1B] = 0xFF;
	memory[0xFF1C] = 0x9F;
	memory[0xFF1D] = 0xFF;
	memory[0xFF1E] = 0xBF;
	memory[0xFF20] = 0xFF;
	memory[0xFF21] = 0x00;
	memory[0xFF22] = 0x00;
	memory[0xFF23] = 0xBF;
	memory[0xFF24] = 0x77;
	memory[0xFF25] = 0xF3;
	memory[0xFF26] = 0xF1;
	memory[0xFF40] = 0x91;
	memory[0xFF41] = 0x85;
	memory[0xFF42] = 0x00;
	memory[0xFF43] = 0x00;
	memory[0xFF44] = 0x00;
	memory[0xFF45] = 0x00;
	memory[0xFF46] = 0xFF;
	memory[0xFF47] = 0xFC;

	// These aren't initialised in the boot rom, but it's better to keep them stable
	memory[0xFF48] = 0x00;
	memory[0xFF49] = 0x00;

	memory[0xFF4A] = 0x00;
	memory[0xFF4B] = 0x00;
	memory[0xFF4D] = 0xFF;
	memory[0xFF4F] = 0xFF;
	memory[0xFF51] = 0xFF;
	memory[0xFF52] = 0xFF;
	memory[0xFF53] = 0xFF;
	memory[0xFF54] = 0xFF;
	memory[0xFF55] = 0xFF;
	memory[0xFF56] = 0xFF;
	memory[0xFF68] = 0xFF;
	memory[0xFF69] = 0xFF;
	memory[0xFF6A] = 0xFF;
	memory[0xFF6B] = 0xFF;
	memory[0xFF70] = 0xFF;
	memory[0xFFFF] = 0x00;
	
	cpu_cycles = 0;
	ppu_cycles = 0;
}

bool ExtractBit(uint8_t value, uint8_t bit)
{
	return (value >> bit) & 0x01;
}

uint8_t GetHighByte(uint16_t value)
{
	return (value & 0xff00) >> 8;
}

uint8_t GetLowByte(uint16_t value)
{
	return value & 0x00ff;
}

void GameBoy::Write(uint8_t value, uint16_t address)
{
	// Cartridge memory
	if (address >= 0x0000 && address <= 0x7FFF)
	{
		return;
	}

	if (address >= 0x8000 && address < 0xA000 && !CanAccessVRAM())
	{
		return;
	}

	if (address >= 0xFEA0 && address <= 0xFEFF)
	{
		return;
	}

	if (address == 0xFF50 && value != 0x00)
	{
		// todo: make this reversible
		UnmapBootRom();
		return;
	}

	// serial transfer output to console
	if (address == 0xFF02 && value == 0x81)
	{
		_ASSERT(serial_transfer.is_open());
		serial_transfer << char(memory[0xFF01]);
		serial_transfer.flush();	
		memory[0xFF02] = 0;
		return;
	}

	// divider register resets when written
	if (address == 0xFF04)
	{
		memory[0xFF04] = 0;
		return;
	}

	// OAM DMA transfer
	if (address == 0xFF46 && !DMA_transfer)
	{
		uint16_t source_address = value * 0x100;
		StartDMATransfer(source_address);
	}
	memory[address] = value;
}

// todo: conditions for correct read
uint8_t GameBoy::Read(uint16_t address)
{
	if (address >= 0xFEA0 && address <= 0xFEFF)
	{
		return 0xFF;
	}

	if (DMA_transfer && !(address >= 0xFF80 && address <= 0xFFFE))
	{
		return 0x00;
	}

	if (address == 0xFF00)
	{
		UpdateJoypadInput();
	}

	return memory[address];
}

void GameBoy::StartDMATransfer(uint16_t address)
{
	DMA_transfer = true;
	DMA_start = cpu_cycles;
	DMA_address = address;
}

void GameBoy::WriteScanlineY(uint8_t value)
{
	// todo: check for access rights
	Write(value, 0xFF44);
}

void GameBoy::UpdatePPU(uint64_t cycleCount)
{ 
	uint64_t previousScanlineNumber = ppu_cycles / 456;
	uint64_t nextScanlineNumber = (ppu_cycles + cycleCount) / 456;

	Mode previousMode = CalculatePPUMode();
	Mode nextMode = CalculatePPUMode(cycleCount);

	uint8_t& control = memory[0xFF40];

	if (!ExtractBit(control, 7))
	{
		return;
	}

	uint8_t& stat = memory[0xFF41];
	uint8_t& interruptRequests = memory[0xFF0F];
	
	stat |= uint8_t(nextMode);
	stat &= (uint8_t(nextMode) | 0b11111100);

	if (nextMode == Mode::PixelTransfer)
	{
		// FIFO stuff
		if (nextScanlineNumber == previousScanlineNumber)
		{
			// fifo stuff for this scanline
		}

		else
		{
			uint16_t previous_fifo_dots = 456 - (ppu_cycles % 456);
			uint16_t next_fifo_dots = (ppu_cycles + cycleCount) % 456;

			// fifo for previous scanline
			// fifo for this scanline
		}
	}


	if (previousMode != nextMode)
	{
		if (nextMode == Mode::Vblank)
		{
			interruptRequests |= (1 << 0);
		}
		if (nextMode == Mode::Hblank && ExtractBit(stat, 3) || nextMode == Mode::Vblank && ExtractBit(stat, 4)
			|| nextMode == Mode::OAMSearch && ExtractBit(stat, 5) || nextMode == Mode::PixelTransfer && ExtractBit(stat, 6))
		{
			interruptRequests |= (1 << 1);
		}
		if (nextMode == Mode::PixelTransfer)
		{
			// pause rendering, discard pixels
		}
	}

	if (nextScanlineNumber != previousScanlineNumber)
	{
		uint8_t LY = memory[0xFF44];
		uint8_t LYC = memory[0xFF45];
		WriteScanlineY(LY + nextScanlineNumber - previousScanlineNumber);

		if (LY == LYC)
		{
			stat |= (1 << 2);
			if (ExtractBit(stat, 6))
			{
				interruptRequests |= (1 << 1);
			}
		}

		else
		{
			stat &= ~(1 << 2);
		}
	}
}

void GameBoy::ExecutionLoop()
{
	while (true)
	{
		/*if (!haltMode)
		{
			LogCPUState();
		}*/
		
		HandleInterrupts();

		uint16_t previousPC = programCounter;
		if (!haltMode)
		{
			uint8_t opcode = ReadROMLine();
			ExecuteInstruction(opcode);
		}
		else
		{
			UpdateClock(4);
		}
	

		if (ppu_cycles >= 70224)
		{
			if (ExtractBit(memory[0xFF40], 7) && ExtractBit(memory[0xFF40], 0))
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

void GameBoy::PointToCartridgeMemory()
{
	programCounter = 0x0100;
}

bool GameBoy::CanAccessVRAM()
{
	uint8_t screenMode = memory[0xFF41] & 0b00000011;

	switch (screenMode)
	{
	case 0:
	case 1:
	case 2:
		return true;
	case 3:
		return false;
	default:
		_ASSERTE(false);
	}
}

bool GameBoy::CanAccessOAM()
{
	uint8_t screenMode = memory[0xFF41] & 0b00000011;

	switch (screenMode)
	{
	case 0:
	case 1:
		return true;
	case 2:
	case 3:
		return false;
	default:
		_ASSERTE(false);
	}
}

void GameBoy::HandleInterrupts()
{
	uint8_t& interruptEnable = memory[0xFFFF];
	uint8_t& interruptRequests = memory[0xFF0F];

	// Todo: STAT interrupt, TIMA interrupt, Serial interrupt, Joypad interrupt
	// Todo: halt should be disabled as soon as an interrupt is pending (not necessarily handled)

	uint8_t interrupts = interruptEnable & interruptRequests;

	if (interrupts)
	{
		haltMode = 0;
	}

	auto activate_interrupt = [&](uint8_t num) {
		_ASSERTE(num < 5 && !DMA_transfer);
		interruptRequests &= ~(1 << num);
		IME = 0;

		Push(GetHighByte(programCounter));
		Push(GetLowByte(programCounter));
		uint8_t interruptVector = 0x40 + num * 0x8;
		programCounter = interruptVector;
				
		UpdateClock(20); // Todo: Add halt mode factor
	};



	for (uint8_t i = 0; i < 5; i++)
	{
		if (ExtractBit(interrupts, i) && IME)
		{
			activate_interrupt(i);
			break;
		}
	}
}

Instruction GameBoy::DecodeOpcode(uint8_t opcode)
{
	if (opcode >= 0x40 && opcode <= 0x7F && opcode != 0x76)
	{
		return Instruction::LD;
	}
	
	if (opcode >= 0x80 && opcode < 0x88)
	{
		return Instruction::ADD;
	}

	if (opcode >= 0x88 && opcode < 0x90)
	{
		return Instruction::ADC;
	}
	
	if (opcode >= 0x90 && opcode < 0x98)
	{
		return Instruction::SUB;
	}

	if (opcode >= 0x98 && opcode < 0xA0)
	{
		return Instruction::SBC;
	}

	if (opcode >= 0xA0 && opcode < 0xA8)
	{
		return Instruction::AND;
	}

	if (opcode >= 0xA8 && opcode < 0xB0)
	{
		return Instruction::XOR;
	}

	if (opcode >= 0xB0 && opcode < 0xB8)
	{
		return Instruction::OR;
	}

	if (opcode >= 0xB8 && opcode < 0xC0)
	{
		return Instruction::CP;
	}

	switch (opcode)
	{
	case 0x00:
		return Instruction::NOP;
	case 0x10:
		return Instruction::STOP;
	case 0x20:
	case 0x30:
		return Instruction::JR_COND;
	case 0x01:
	case 0x11:
	case 0x21:
	case 0x31:
		return Instruction::LD_16_IMMEDIATE;
	case 0x02:
	case 0x12:
	case 0x22:
	case 0x32:
		return Instruction::LD_TO_INDIRECT;
	case 0x03:
	case 0x13:
	case 0x23:
	case 0x33:
		return Instruction::INC_16;
	case 0x04:
	case 0x14:
	case 0x24:
	case 0x34:
		return Instruction::INC;
	case 0x05:
	case 0x15:
	case 0x25:
	case 0x35:
		return Instruction::DEC;
	case 0x06:
	case 0x16:
	case 0x26:
	case 0x36:
		return Instruction::LD_IMMEDIATE;
	case 0x07:
		return Instruction::RLCA;
	case 0x17:
		return Instruction::RLA;
	case 0x27:
		return Instruction::DAA;
	case 0x37:
		return Instruction::SCF;
	case 0x08:
		return Instruction::LD_A16;
	case 0x18:
		return Instruction::JR;
	case 0x28:
	case 0x38:
		return Instruction::JR_COND;
	case 0x09:
	case 0x19:
	case 0x29:
	case 0x39:
		return Instruction::ADD_16;
	case 0x0A:
	case 0x1A:
	case 0x2A:
	case 0x3A:
		return Instruction::LD_INDIRECT;
	case 0x0B:
	case 0x1B:
	case 0x2B:
	case 0x3B:
		return Instruction::DEC_16;
	case 0x0C:
	case 0x1C:
	case 0x2C:
	case 0x3C:
		return Instruction::INC;
	case 0x0D:
	case 0x1D:
	case 0x2D:
	case 0x3D:
		return Instruction::DEC;
	case 0x0E:
	case 0x1E:
	case 0x2E:
	case 0x3E:
		return Instruction::LD_IMMEDIATE;
	case 0x0F:
		return Instruction::RRCA;
	case 0x1F:
		return Instruction::RRA;
	case 0x2F:
		return Instruction::CPL;
	case 0x3F:
		return Instruction::CCF;
	case 0xC0:
	case 0xD0:
		return Instruction::RET_COND;
	case 0xE0:
	case 0xF0:
		return Instruction::LDH;
	case 0xC1:
	case 0xD1:
	case 0xE1:
	case 0xF1:
		return Instruction::POP_16;
	case 0xC2:
	case 0xD2:
		return Instruction::JP_COND; // unimplemented
	case 0xE2:
		return Instruction::LD_FROM_ACC;
	case 0xF2:
		return Instruction::LD_TO_ACC;
	case 0xC3:
		return Instruction::JP; // unimplemented
	case 0xF3:
		return Instruction::DI;
	case 0xC4:
	case 0xD4:
		return Instruction::CALL_COND;
	case 0xC5:
	case 0xD5:
	case 0xE5:
	case 0xF5:
		return Instruction::PUSH_16;
	case 0xC6:
		return Instruction::ADD_IMMEDIATE;
	case 0xD6:
		return Instruction::SUB_IMMEDIATE;
	case 0xE6:
		return Instruction::AND_IMMEDIATE;
	case 0xF6:
		return Instruction::OR_IMMEDIATE;
	case 0xC7:
	case 0xD7:
	case 0xE7:
	case 0xF7:
		return Instruction::RST;
	case 0xC8:
	case 0xD8:
		return Instruction::RET_COND;
	case 0xE8:
		return Instruction::ADD_SP_IMMEDIATE;
	case 0xF8:
		return Instruction::LD_HL_FROM_SP;
	case 0xC9:
		return Instruction::RET;
	case 0xD9:
		return Instruction::RETI;
	case 0xE9:
		return Instruction::JP_HL;
	case 0xF9:
		return Instruction::LD_SP_FROM_HL;
	case 0xCA:
	case 0xDA:
		return Instruction::JP_COND;
	case 0xEA:
		return Instruction::LD_TO_ADDRESS;
	case 0xFA:
		return Instruction::LD_FROM_ADDRESS;
	case 0xCB:
		return Instruction::PREFIX_CB;
	case 0xFB:
		return Instruction::EI;
	case 0xCC:
	case 0xDC:
		return Instruction::CALL_COND;
	case 0xCD:
		return Instruction::CALL;
	case 0xCE:
		return Instruction::ADC_IMMEDIATE;
	case 0xDE:
		return Instruction::SBC_IMMEDIATE;
	case 0xEE:
		return Instruction::XOR_IMMEDIATE;
	case 0xFE:
		return Instruction::CP_IMMEDIATE;
	case 0xCF:
	case 0xDF:
	case 0xEF:
	case 0xFF:
		return Instruction::RST;
	case 0x76:
		return Instruction::HALT;
	default:
		_ASSERT(false);
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
			memory[startAddress + byteNumber] = bytes[byteNumber];
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
		for (uint32_t byteNumber = 0; byteNumber < 0x100; byteNumber++)
		{
			cartridgeMemory[byteNumber] = bytes[byteNumber];
		}

		for (uint32_t byteNumber = 0x100; byteNumber < bytes.size(); byteNumber++)
		{
			memory[byteNumber] = bytes[byteNumber];
		}
		return;
	}
	_ASSERTE(false);
}

void GameBoy::UnmapBootRom()
{
	for (uint32_t byteNumber = 0; byteNumber < 0x100; byteNumber++)
	{
		memory[byteNumber] = cartridgeMemory[byteNumber];
	}
}

void GameBoy::IncrementProgramCounter(int8_t offset)
{
	programCounter += offset;
	UpdateClock(4);
}

void GameBoy::SetProgramCounter(uint16_t value)
{
	programCounter = value;
	UpdateClock(4);
}

uint8_t GameBoy::GetColor(uint8_t index)
{
	uint8_t palette = memory[0xFF47];
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
	uint8_t LCDControl = memory[0xFF40];
	uint16_t startAddress = ExtractBit(LCDControl, 3) ? 0x9C00 : 0x9800;
	std::array<uint8_t, 256 * 256> backgroundColors;
	for (uint32_t tileNum = 0; tileNum < 32 * 32; tileNum++)
	{
		std::array<uint8_t, 64> tileData = ExtractTileData(memory[startAddress + tileNum]);
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
		sprite_vec.emplace_back(memory[address], memory[address + 1], memory[address + 2], memory[address + 3]);
	}

	std::array<uint8_t, 256 * 256> sprite_colors;

	// Unused space is -1
	// todo: std optional?
	for (auto& element : sprite_colors)
	{
		element = -1;
	}

	uint8_t SCY = memory[0xFF42];
	uint8_t SCX = memory[0xFF43];

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
	uint8_t SCY = memory[0xFF42];
	uint8_t SCX = memory[0xFF43];

	SDL_RenderSetLogicalSize(renderer, 160, 144);
	SDL_RenderClear(renderer);

	// Background/Window
	for (uint8_t offsetX = 0; offsetX < 160; offsetX++)
	{
		for (uint8_t offsetY = 0; offsetY < 144; offsetY++)
		{
			uint8_t color = 255 - 85 * backgroundColors[(SCX + offsetX) + 256 *  (SCY + offsetY )];
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
			uint8_t color_index = objectColors[(SCX + offsetX) + 256 * (SCY + offsetY)];
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
		backgroundMap.push_back(memory[i]);
	}

	for (uint16_t i = 0x8000; i < 0x8800; i++)
	{
		tileMap.push_back(memory[i]);
	}

	for (uint16_t i = 0x104; i < 0x134; i++)
	{
		logo.push_back(memory[i]);
	}

	return;
}

std::array<uint8_t, 64> GameBoy::ExtractTileData(uint8_t offset)
{
	uint8_t LCDControl = memory[0xFF40];
	int8_t signed_offset = offset;
	bool mapSelect = ExtractBit(LCDControl, 4);
	uint16_t tileAddress = mapSelect ?  0x8000 + 16 * offset : 0x9000 + signed_offset * 16;
	std::array<uint8_t, 64> colorSelect;

	for (uint8_t pixelNum = 0; pixelNum < 64; pixelNum++)
	{
		uint8_t bit = 7 - (pixelNum % 8);
		uint8_t rowNum = pixelNum / 8;
		uint8_t paletteChoice = uint8_t(ExtractBit(memory[tileAddress + 2 * rowNum], bit)) 
			+ uint8_t(ExtractBit(memory[tileAddress + 2 * rowNum + 1], bit) << 1);
		colorSelect[pixelNum] = paletteChoice;
	}

	return colorSelect;
}

std::array<uint8_t, 64> GameBoy::ExtractTileDataUnsigned(uint8_t offset)
{
	uint8_t LCDControl = memory[0xFF40];
	uint16_t tileAddress = 0x8000 + 16 * offset;
	std::array<uint8_t, 64> colorSelect;

	for (uint8_t pixelNum = 0; pixelNum < 64; pixelNum++)
	{
		uint8_t bit = 7 - (pixelNum % 8);
		uint8_t rowNum = pixelNum / 8;
		uint8_t paletteChoice = uint8_t(ExtractBit(memory[tileAddress + 2 * rowNum], bit))
			+ uint8_t(ExtractBit(memory[tileAddress + 2 * rowNum + 1], bit) << 1);
		colorSelect[pixelNum] = paletteChoice;
	}

	return colorSelect;
}

uint8_t GameBoy::ReadROMLine()
{
	UpdateClock(4);
	return memory[programCounter++];
}

void GameBoy::Push(uint8_t value)
{
	stackPointer--;
	Write(value, stackPointer);
	UpdateClock(4);
}

void GameBoy::Push16(uint8_t opcode)
{
	uint16_t value;
	std::string reg_string;

	switch (opcode)
	{
	case 0xC5:
		value = Get16BitRegisterValue(BC16);
		reg_string = "BC";
		break;
	case 0xD5:
		value = Get16BitRegisterValue(DE16);
		reg_string = "DE";
		break;
	case 0xE5:
		value = Get16BitRegisterValue(HL16);
		reg_string = "HL";
		break;
	case 0xF5:
		value = (uint16_t(registerA) << 8) + registerF;
		reg_string = "AF";
		break;
	default:
		_ASSERTE(false);
	}

	Push(GetHighByte(value));
	Push(GetLowByte(value));

	if (printLogs)
	{
		std::cout << "PUSH " << reg_string << "\n";
	}
}

uint8_t GameBoy::Pop()
{
	uint8_t result = memory[stackPointer];
	stackPointer++;
	UpdateClock(4);
	return result;
}

void GameBoy::Pop16(uint8_t opcode)
{	
	uint8_t lsb = Pop();
	uint8_t msb = Pop();
	std::string reg_string;
	
	switch (opcode)
	{
	case 0xC1:
		registerB = msb;
		registerC = lsb;
		reg_string = "BC";
		break;
	case 0xD1:
		registerD = msb;
		registerE = lsb;
		reg_string = "DE";
		break;
	case 0xE1:
		registerH = msb;
		registerL = lsb;
		reg_string = "HL";
		break;
	case 0xF1:
		registerA = msb;
		registerF = lsb & 0xf0;
		reg_string = "AF";
		break;
	default:
		_ASSERT(false);
	}

	if (printLogs)
	{
		std::cout << "POP " << reg_string << "\n";
	}
}

uint8_t GameBoy::GetRegisterValue(uint8_t num)
{
	switch (num)
	{
	case B:
		return registerB;
	case C:
		return registerC;
	case D:
		return registerD;
	case E:
		return registerE;
	case H:
		return registerH;
	case L:
		return registerL;
	case HL:
		return memory[Get16BitRegisterValue(HL16)];
	case A:
		return registerA;
	}
	_ASSERTE(false);
	return 0;
}

uint16_t GameBoy::Get16BitRegisterValue(uint8_t num)
{
	UpdateClock(4);
	switch (num)
	{
	case BC16:
		return uint16_t(registerB << 8) + registerC;
	case DE16:
		return uint16_t(registerD << 8) + registerE;
	case HL16:
		return uint16_t(registerH << 8) + registerL;
	case SP16:
		return stackPointer;
	}
	_ASSERTE(false);
	return 0;
}

void GameBoy::SetRegister(uint8_t num, uint16_t value)
{
	switch (num)
	{
	case B:
		registerB = uint8_t(value);
		return;
	case C:
		registerC = uint8_t(value);
		return;
	case D:
		registerD = uint8_t(value);
		return;
	case E:
		registerE = uint8_t(value);
		return;
	case H:
		registerH = uint8_t(value);
		return;
	case L:
		registerL = uint8_t(value);
		return;
	case HL:
		Write(uint8_t(value), Get16BitRegisterValue(HL16));
		return;
	case A:
		registerA = uint8_t(value);
		return;
	}
	_ASSERTE(false);
}

void GameBoy::Set16BitRegister(uint8_t num, uint16_t value)
{
	switch (num)
	{
	case BC16:
		registerB = GetHighByte(value);
		registerC = GetLowByte(value);
		return;
	case DE16:
		registerD = GetHighByte(value);
		registerE = GetLowByte(value);
		return;
	case HL16:
		registerH = GetHighByte(value);
		registerL = GetLowByte(value);
		return;
	case SP16:
		stackPointer = value;
		return;
	}

	_ASSERTE(false);
}
uint8_t GameBoy::CheckCondition(uint8_t condition)
{
	_ASSERTE(condition < 4);

	switch (condition)
	{
	case 0:
		return !GetFlag(ZeroFlag);
	case 1:
		return GetFlag(ZeroFlag);
	case 2:
		return !GetFlag(CarryFlag);
	case 3:
		return GetFlag(CarryFlag);
	}
	return false;
}

void GameBoy::ExecuteInstruction(uint8_t opcode)
{
	Instruction instruction = DecodeOpcode(opcode);
	if (printLogs)
	{
		std::cout << std::format("{:X}", programCounter - 1) << "   -->   ";
	}

	switch (instruction)
	{
	case Instruction::NOP:
		Nop(opcode);
		return;
	case Instruction::STOP:
		Stop(opcode);
		return;
	case Instruction::ADD:
		Add(opcode);
		return;
	case Instruction::ADD_16:
		Add16(opcode);
		return;
	case Instruction::ADD_IMMEDIATE:
		ImmediateAdd(opcode);
		return;
	case Instruction::ADD_SP_IMMEDIATE:
		ImmediateAddToStackPointer(opcode);
		return;
	case Instruction::ADC:
		Adc(opcode);
		return;
	case Instruction::ADC_IMMEDIATE:
		ImmediateAdc(opcode);
		return;
	case Instruction::AND:
		And(opcode);
		return;
	case Instruction::AND_IMMEDIATE:
		ImmediateAnd(opcode);
		return;
	case Instruction::JR:
		RelativeJump(opcode);
		return;
	case Instruction::JR_COND:
		ConditionalRelativeJump(opcode);
		return;
	case Instruction::LD:
		LD(opcode);
		return;
	case Instruction::LDH:
		LDH(opcode);
		return;
	case Instruction::LD_IMMEDIATE:
		ImmediateLD(opcode);
		return;
	case Instruction::LD_16_IMMEDIATE:
		ImmediateLD16(opcode);
		return;
	case Instruction::LD_A16:
		LDStackPointertoAddress(opcode);
		return;
	case Instruction::LD_INDIRECT:
		IndirectLD(opcode);
		return;
	case Instruction::LD_TO_INDIRECT:
		LDtoIndirect(opcode);
		return;
	case Instruction::LD_TO_ACC:
		LDAccumulator(opcode);
		return;
	case Instruction::LD_FROM_ACC:
		LDfromAccumulator(opcode);
		return;
	case Instruction::LD_TO_ADDRESS:
		LDtoAddress(opcode);
		return;
	case Instruction::LD_FROM_ADDRESS:
		LDfromAddress(opcode);
		return;
	case Instruction::LD_SP_FROM_HL:
		LDStackPointerFromHL(opcode);
		return;
	case Instruction::LD_HL_FROM_SP:
		LDHLFromStackPointer(opcode);
		return;
	case Instruction::SUB:
		Sub(opcode);
		return;
	case Instruction::SUB_IMMEDIATE:
		ImmediateSub(opcode);
		return;
	case Instruction::SBC:
		SBC(opcode);
		return;
	case Instruction::INC:
		Increment(opcode);
		return;
	case Instruction::INC_16:
		Increment16(opcode);
		return;
	case Instruction::DEC:
		Decrement(opcode);
		return;
	case Instruction::DEC_16:
		Decrement16(opcode);
		return;
	case Instruction::OR:
		Or(opcode);
		return;
	case Instruction::OR_IMMEDIATE:
		ImmediateOr(opcode);
		return;
	case Instruction::XOR:
		Xor(opcode);
		return;
	case Instruction::XOR_IMMEDIATE:
		ImmediateXor(opcode);
		return;
	case Instruction::CP:
		Compare(opcode);
		return;
	case Instruction::CP_IMMEDIATE:
		ImmediateCompare(opcode);
		return;
	case Instruction::JP:
		Jump(opcode);
		return;
	case Instruction::JP_COND:
		ConditionalJump(opcode);
		return;
	case Instruction::JP_HL:
		JumpToHL(opcode);
		return;
	case Instruction::CALL:
		FunctionCall(opcode);
		return;
	case Instruction::CALL_COND:
		ConditionalFunctionCall(opcode);
		return;
	case Instruction::RET:
		FunctionReturn(opcode);
		return;
	case Instruction::RET_COND:
		ConditionalFunctionReturn(opcode);
		return;
	case Instruction::RETI:
		InterruptReturn(opcode);
		return;
	case Instruction::EI:
		EnableInterrupts(opcode);
		return;
	case Instruction::RST:
		Restart(opcode);
		return;
	case Instruction::DI:
		DisableInterrupts(opcode);
		return;
	case Instruction::RLA:
		RotateLeftAccumulator(opcode);
		return;
	case Instruction::RRA:
		RotateRightAccumulator(opcode);
		return;
	case Instruction::PUSH_16:
		Push16(opcode);
		return;
	case Instruction::POP_16:
		Pop16(opcode);
		return;
	case Instruction::CPL:
		CPL(opcode);
		return;
	case Instruction::SCF:
		SCF(opcode);
		return;
	case Instruction::CCF:
		CCF(opcode);
		return;
	case Instruction::RLCA:
		RLCA(opcode);
		return;
	case Instruction::RRCA:
		RRCA(opcode);
		return;
	case Instruction::DAA:
		DAA(opcode);
		return;
	case Instruction::SBC_IMMEDIATE:
		SBCImmediate(opcode);
		return;
	case Instruction::HALT:
		Halt(opcode);
		return;
	case Instruction::PREFIX_CB:
		ExecuteCBInstruction();
		return;
	}
	_ASSERTE(false);
	return;
}


Instruction GameBoy::DecodeCBOpcode(uint8_t opcode)
{
	if (opcode >= 0x00 && opcode < 0x08)
	{
		return Instruction::RLC;
	}

	if (opcode >= 0x08 && opcode < 0x10)
	{
		return Instruction::RRC;
	}

	if (opcode >= 0x10 && opcode < 0x18)
	{
		return Instruction::RL;
	}

	if (opcode >= 0x18 && opcode < 0x20)
	{
		return Instruction::RR;
	}

	if (opcode >= 0x20 && opcode < 0x28)
	{
		return Instruction::SLA;
	}

	if (opcode >= 0x28 && opcode < 0x30)
	{
		return Instruction::SRA;
	}

	if (opcode >= 0x30 && opcode < 0x38)
	{
		return Instruction::SWAP;
	}

	if (opcode >= 0x38 && opcode < 0x40)
	{
		return Instruction::SRL;
	}

	if (opcode >= 0x40 && opcode < 0x80)
	{
		return Instruction::BIT;
	}

	if (opcode >= 0x80 && opcode < 0xC0)
	{
		return Instruction::RES;
	}

	if (opcode >= 0xC0)
	{
		return Instruction::SET;
	}

	_ASSERT(false);
}
void GameBoy::ExecuteCBInstruction()
{
	uint8_t opcode = ReadROMLine();
	Instruction instruction = DecodeCBOpcode(opcode);
	if (printLogs)
	{
		std::cout << "CB: ";
	}

	switch (instruction)
	{
	case Instruction::RLC:
		RotateLeftCircular(opcode);
		return;
	case Instruction::RRC:
		RotateRightCircular(opcode);
		return;
	case Instruction::RL:
		RotateLeft(opcode);
		return;
	case Instruction::RR:
		RotateRight(opcode);
		return;
	case Instruction::SLA:
		SLA(opcode);
		return;
	case Instruction::SRA:
		SRA(opcode);
		return;
	case Instruction::SWAP:
		SWAP(opcode);
		return;
	case Instruction::SRL:
		ShiftRightThroughCarry(opcode);
		return;
	case Instruction::BIT:
		Bit(opcode);
		return;
	case Instruction::RES:
		RES(opcode);
		return;
	case Instruction::SET:
		SET(opcode);
		return;
	}
	_ASSERTE(false);
}

void GameBoy::LD(uint8_t opcode)
{
	uint8_t reg1 = (opcode & 0b00111000) >> 3;
	uint8_t reg2 = opcode & 0b00000111;
	SetRegister(reg1, GetRegisterValue(reg2));

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), reg1, reg2);
	}
}

void GameBoy::LDH(uint8_t opcode)
{
	uint8_t offset = ReadROMLine();

	if (opcode == 0xE0)
	{
		Write(GetRegisterValue(A), 0xFF00 + offset);
		if (printLogs)
		{
			std::cout << InstructionToString(DecodeOpcode(opcode))
				<< std::format(" ${:X}, ",0xFF00 + offset) << RegisterToString(A) << "\n";
		}
	}

	if (opcode == 0xF0)
	{
		SetRegister(A, Read(0xFF00 + offset));
		if (printLogs)
		{
			PrintImmediateInstruction(DecodeOpcode(opcode), 0xFF00 + offset, A);
		}
	}
}

void GameBoy::LDtoIndirect(uint8_t opcode)
{
	std::string reg_string;
	switch (opcode)
	{
	case 0x02:
		Write(registerA, Get16BitRegisterValue(BC16));
		reg_string = "(BC)";
		break;
	case 0x12:
		Write(registerA, Get16BitRegisterValue(DE16));
		reg_string = "(DE)";
		break;
	case 0x22:
		Write(registerA, Get16BitRegisterValue(HL16));
		Set16BitRegister(HL16, Get16BitRegisterValue(HL16) + 1);
		reg_string = "(HL+)";
		break;
	case 0x32:
		Write(registerA, Get16BitRegisterValue(HL16));
		Set16BitRegister(HL16, Get16BitRegisterValue(HL16) - 1);
		reg_string = "(HL-)";
		break;
	default:
		_ASSERT(false);
	}
	if (printLogs)
	{
		std::cout << InstructionToString(DecodeOpcode(opcode)) << " " << reg_string << ", A" << "\n";
	}
}

void GameBoy::ImmediateLD16(uint8_t opcode)
{
	uint8_t reg = (opcode & 0b00110000) >> 4;
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t immediateValue = (uint16_t(msb) << 8) + lsb;
	Set16BitRegister(reg, immediateValue);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, reg);
	}
}


void GameBoy::IndirectLD(uint8_t opcode)
{
	std::string reg_string;
	switch (opcode)
	{
	case 0x0A:
		registerA = memory[Get16BitRegisterValue(BC16)];
		reg_string = "(BC)";
		break;
	case 0x1A:
		registerA = memory[Get16BitRegisterValue(DE16)];
		reg_string = "(DE)";
		break;
	case 0x2A:
		registerA = memory[Get16BitRegisterValue(HL16)];
		Set16BitRegister(HL16, Get16BitRegisterValue(HL16) + 1);
		reg_string = "(HL+)";
		break;
	case 0x3A:
		registerA = memory[Get16BitRegisterValue(HL16)];
		Set16BitRegister(HL16, Get16BitRegisterValue(HL16) - 1);
		reg_string = "(HL-)";
		break;
	default:
		_ASSERT(false);
	}

	if (printLogs)
	{
		std::cout << InstructionToString(DecodeOpcode(opcode)) << " A, " << reg_string << "\n";
	}
}

void GameBoy::LDfromAccumulator(uint8_t opcode)
{
	uint16_t address = 0xFF00 + GetRegisterValue(C);
	Write(GetRegisterValue(A), address);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), A, C);
	}
}

void GameBoy::LDAccumulator(uint8_t opcode)
{
	uint16_t address = 0xFF00 + GetRegisterValue(C); 
	SetRegister(A, Read(address));

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), C, A);
	}
}

void GameBoy::LDtoAddress(uint8_t opcode)
{
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	Write(registerA, address);
	UpdateClock(4);

	if (printLogs)
	{
		PrintImmediateInstructionReversed(DecodeOpcode(opcode), address, A);
	}
}

void GameBoy::LDStackPointertoAddress(uint8_t opcode)
{
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	Write(GetLowByte(stackPointer), address);
	Write(GetHighByte(stackPointer), address + 1);
	UpdateClock(8);

	if (printLogs)
	{
		std::cout << "LD " << std::format("${:X}, ", address) << "SP" << "\n";
	}
}

// todo: clean up/separate writes to memory
void GameBoy::LDfromAddress(uint8_t opcode)
{
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	SetRegister(A, memory[address]);
	UpdateClock(4);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), address, A);
	}
}

void GameBoy::LDStackPointerFromHL(uint8_t opcode)
{
	stackPointer = Get16BitRegisterValue(HL16);
	UpdateClock(4);

	if (printLogs)
	{
		std::cout << "LD SP, HL" << "\n";
	}
}

void GameBoy::LDHLFromStackPointer(uint8_t opcode)
{
	int8_t offset = ReadROMLine();
	uint32_t result = stackPointer + offset;
	Set16BitRegister(HL16, stackPointer + offset);
	UpdateClock(4);

	bool halfCarry = ((stackPointer & 0xF) + (offset & 0xF)) & 0x10;
	bool carry = ((stackPointer & 0xFF) + uint8_t(offset)) > 0xFF;

	// Todo: figure out flags
	SetFlag(ZeroFlag, 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, carry);

	if (printLogs)
	{
		std::cout << "LD HL, SP + " << std::format("${:X}", offset) << "\n";
	}
}



void GameBoy::Add(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;

	uint8_t Acc = GetRegisterValue(A);
	uint8_t regValue = GetRegisterValue(reg);

	uint16_t result = Acc + regValue;
	bool halfCarry = ((Acc & 0xF) + (regValue & 0xF)) & 0x10;

	SetRegister(A, result);
	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), reg);
	}
}

void GameBoy::Add16(uint8_t opcode)
{
	uint8_t reg = (opcode & 0b01110000) >> 4;

	uint16_t HLValue = Get16BitRegisterValue(HL16);
	uint16_t regValue = Get16BitRegisterValue(reg);

	uint16_t result = HLValue + regValue;

	// Half carry needs to be calculated from bit 11
	uint16_t halfCarryMask = (1 << 12) - 1;
	bool halfCarry = ((HLValue & halfCarryMask) + (regValue & halfCarryMask)) & (1 << 12);

	Set16BitRegister(HL16, result);

	// Zero flag not affected
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, uint32_t(HLValue) + uint32_t(regValue) > 0xFFFF);

	if (printLogs)
	{
		PrintInstruction16(DecodeOpcode(opcode), HL16, reg);
	}
}

void GameBoy::ImmediateAdd(uint8_t opcode)
{
	uint8_t immediateValue = ReadROMLine();
	uint8_t Acc = GetRegisterValue(A);

	uint16_t result = Acc + immediateValue;
	bool halfCarry = ((Acc & 0xF) + (immediateValue & 0xF)) & 0x10;

	SetRegister(A, result);
	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, A);
	}
}

void GameBoy::ImmediateAddToStackPointer(uint8_t opcode)
{
	int8_t offset = ReadROMLine();
	uint16_t oldStackPointer = stackPointer;
	stackPointer += offset;

	bool halfCarry = ((oldStackPointer & 0xF) + (offset & 0xF)) & 0x10;
	bool carry = ((oldStackPointer & 0xFF) + uint8_t(offset)) > 0xFF;

	SetFlag(ZeroFlag, 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, carry);


	if (printLogs)
	{
		std::cout << "ADD SP, " << std::format("${:X}", offset) << "\n";
	}
}

void GameBoy::Adc(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t Acc = GetRegisterValue(A);
	uint8_t carry = GetFlag(CarryFlag);
	uint8_t regValue = GetRegisterValue(reg);

	uint16_t result = Acc + carry + regValue;
	bool halfCarry = ((Acc & 0xF) + (carry & 0xF) + (regValue & 0xF)) & 0x10;

	SetRegister(A, result);
	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), reg);
	}
}

void GameBoy::ImmediateAdc(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t Acc = GetRegisterValue(A);
	uint8_t carry = GetFlag(CarryFlag);
	uint8_t immediateValue = ReadROMLine();

	uint16_t result = Acc + carry + immediateValue;
	bool halfCarry = ((Acc & 0xF) + (carry & 0xF) + (immediateValue & 0xF)) & 0x10;

	SetRegister(A, result);
	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, A);
	}
}

void GameBoy::SetFlag(uint8_t num, bool value)
{
	registerF &= ~(1 << num);
	registerF |= (uint8_t(value) << num);
}

bool GameBoy::GetFlag(uint8_t num)
{
	return registerF & (1 << num);
}

void GameBoy::Sub(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t Acc = GetRegisterValue(A);
	uint8_t regValue = GetRegisterValue(reg);

	uint16_t result = Acc - regValue;
	bool halfCarry = ((Acc & 0xF) - (regValue & 0xF)) & 0x10;
	
	SetRegister(A, result);

	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 1);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), reg);
	}
}

void GameBoy::ImmediateSub(uint8_t opcode)
{
	uint8_t Acc = GetRegisterValue(A);
	uint8_t immediateValue = ReadROMLine();
	
	uint16_t result = Acc - immediateValue;
	bool halfCarry = ((Acc & 0xF) - (immediateValue & 0xF)) & 0x10;

	SetRegister(A, result);

	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 1);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, A);
	}
}

void GameBoy::SBC(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t Acc = GetRegisterValue(A);
	uint8_t regValue = GetRegisterValue(reg);

	uint16_t result = Acc - regValue - GetFlag(CarryFlag);
	bool halfCarry = ((Acc & 0xF) - (regValue & 0xF) - GetFlag(CarryFlag)) & 0x10;

	SetRegister(A, result);

	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 1);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), reg);
	}
}

void GameBoy::Compare(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t Acc = GetRegisterValue(A);
	uint8_t regValue = GetRegisterValue(reg);

	uint16_t result = Acc - regValue;
	bool halfCarry = ((Acc & 0xF) - (regValue & 0xF)) & 0x10;

	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 1);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), reg);
	}
}

void GameBoy::ImmediateCompare(uint8_t opcode)
{
	uint8_t Acc = GetRegisterValue(A);
	uint8_t immediateValue = ReadROMLine();

	uint16_t result = Acc - immediateValue;
	bool halfCarry = ((Acc & 0xF) - (immediateValue & 0xF)) & 0x10;

	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 1);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, A);
	}
}

void GameBoy::RotateLeftAccumulator(uint8_t opcode)
{
	RotateLeftThroughCarry(A);
	SetFlag(ZeroFlag, 0); // TODO: FIX
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		std::cout << "RLA" << "\n";
	}
}

void GameBoy::RotateRightAccumulator(uint8_t opcode)
{
	RotateRightThroughCarry(A);
	SetFlag(ZeroFlag, 0); // TODO: CHECK IF THIS IS NEEDED
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		std::cout << "RRA" << "\n";
	}
}

void GameBoy::RotateLeft(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	RotateLeftThroughCarry(reg);
	SetFlag(ZeroFlag, GetRegisterValue(reg) == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::RotateRight(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	RotateRightThroughCarry(reg);
	SetFlag(ZeroFlag, GetRegisterValue(reg) == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::RotateLeftThroughCarry(uint8_t reg)
{
	uint8_t oldValue = GetRegisterValue(reg);
	bool oldCarry = GetFlag(CarryFlag);
	SetFlag(CarryFlag, ExtractBit(oldValue, 7));
	SetRegister(reg, (oldValue << 1) + uint8_t(oldCarry));
}

void GameBoy::RotateRightThroughCarry(uint8_t reg)
{
	uint8_t oldValue = GetRegisterValue(reg);
	bool oldCarry = GetFlag(CarryFlag);
	SetFlag(CarryFlag, ExtractBit(oldValue, 0));
	SetRegister(reg, (oldValue >> 1) + (uint8_t(oldCarry) << 7));
}

void GameBoy::RotateLeftCircular(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);

	SetFlag(CarryFlag, ExtractBit(oldValue, 7));
	SetRegister(reg, std::rotl(oldValue, 1));
	SetFlag(ZeroFlag, GetRegisterValue(reg) == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);
	
	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::RotateRightCircular(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);

	SetFlag(CarryFlag, ExtractBit(oldValue, 0));
	SetRegister(reg, std::rotr(oldValue, 1));
	SetFlag(ZeroFlag, GetRegisterValue(reg) == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::RLCA(uint8_t opcode)
{
	uint8_t oldValue = GetRegisterValue(A);

	SetFlag(CarryFlag, ExtractBit(oldValue, 7));
	SetRegister(A, std::rotl(oldValue, 1));
	SetFlag(ZeroFlag, false); // TODO: figure out actual behavior
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), A);
	}
}

void GameBoy::RRCA(uint8_t opcode)
{
	uint8_t oldValue = GetRegisterValue(A);

	SetFlag(CarryFlag, ExtractBit(oldValue, 0));
	SetRegister(A, std::rotr(oldValue, 1));
	SetFlag(ZeroFlag, false); // TODO: fix
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), A);
	}
}

void GameBoy::DAA(uint8_t opcode)
{
	if (!GetFlag(NegativeFlag))
	{
		if (GetFlag(CarryFlag) || registerA > 0x99)
		{
			registerA += 0x60;
			SetFlag(CarryFlag, true);
		}

		if (GetFlag(HalfCarryFlag) || (registerA & 0x0f) > 0x09)
		{
			registerA += 0x06;
		}
	}

	else
	{
		if (GetFlag(CarryFlag))
		{
			registerA -= 0x60;
		}

		if (GetFlag(HalfCarryFlag))
		{
			registerA -= 0x06;
		}
	}

	SetFlag(ZeroFlag, registerA == 0);
	SetFlag(HalfCarryFlag, false);
	UpdateClock(4);

	if (printLogs)
	{
		std::cout << "DAA\n";
	}
}

void GameBoy::SBCImmediate(uint8_t opcode)
{
	uint8_t Acc = GetRegisterValue(A);
	uint8_t immediateValue = ReadROMLine();

	uint16_t result = Acc - immediateValue - GetFlag(CarryFlag);
	bool halfCarry = ((Acc & 0xF) - (immediateValue & 0xF) - GetFlag(CarryFlag)) & 0x10;

	SetRegister(A, result);

	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 1);
	SetFlag(HalfCarryFlag, halfCarry);
	SetFlag(CarryFlag, result > 0xFF);
	UpdateClock(4);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, A);
	}
}

void GameBoy::ShiftRightThroughCarry(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);	
	uint8_t newValue = oldValue >> 1;

	SetRegister(reg, newValue);
	SetFlag(CarryFlag, ExtractBit(oldValue, 0));
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::Increment(uint8_t opcode)
{
	uint8_t reg = (opcode & 0b00111000) >> 3;
	uint8_t regValue = GetRegisterValue(reg);
	
	uint16_t result = regValue + 1;
	bool halfCarry = ((regValue & 0xF) + 1) & 0x10;

	SetRegister(reg, result);
	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, halfCarry);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), reg);
	}
}

void GameBoy::Increment16(uint8_t opcode)
{
	uint8_t reg = (opcode & 0b00110000) >> 4;
	uint16_t value = Get16BitRegisterValue(reg);
	Set16BitRegister(reg, value + 1);

	if (printLogs)
	{
		PrintInstruction16(DecodeOpcode(opcode), reg);
	}
}

void GameBoy::Decrement(uint8_t opcode)
{
	uint8_t reg = (opcode & 0b00111000) >> 3;
	uint8_t regValue = GetRegisterValue(reg);

	uint8_t result = regValue - 1;
	bool halfCarry = ((regValue & 0xF) - 1) & 0x10;

	SetRegister(reg, result);
	SetFlag(ZeroFlag, uint8_t(result) == 0);
	SetFlag(NegativeFlag, 1);
	SetFlag(HalfCarryFlag, halfCarry);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), reg);
	}
}

void GameBoy::Decrement16(uint8_t opcode)
{
	uint8_t reg = (opcode & 0b00110000) >> 4;
	uint16_t value = Get16BitRegisterValue(reg);
	std::string reg_string;
	Set16BitRegister(reg, value - 1);

	switch (reg)
	{
	case 0:
		reg_string = "BC";
	case 1:
		reg_string = "DE";
	case 2:
		reg_string = "HL";
	case 3:
		reg_string = "SP";
	}

	if (printLogs)
	{
		std::cout << "DEC16 " << reg_string << "\n";
	}
}

void GameBoy::And(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t Acc = GetRegisterValue(A);
	uint8_t regValue = GetRegisterValue(reg);

	uint8_t result = Acc & regValue;
	
	SetRegister(A, result);
	SetFlag(ZeroFlag, result == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 1);
	SetFlag(CarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), A, reg);
	}
}

void GameBoy::ImmediateAnd(uint8_t opcode)
{
	uint8_t immediateValue = ReadROMLine();
	uint8_t Acc = GetRegisterValue(A);
	
	uint8_t result = Acc & immediateValue;

	SetRegister(A, result);
	SetFlag(ZeroFlag, result == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 1);
	SetFlag(CarryFlag, 0);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, A);
	}
}
void GameBoy::Or(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t Acc = GetRegisterValue(A);
	uint8_t regValue = GetRegisterValue(reg);

	uint8_t result = Acc | regValue;

	SetRegister(A, result);
	SetFlag(ZeroFlag, result == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);
	SetFlag(CarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), A, reg);
	}
}

void GameBoy::ImmediateOr(uint8_t opcode)
{
	uint8_t immediateValue = ReadROMLine();
	uint8_t Acc = GetRegisterValue(A);

	uint8_t result = Acc | immediateValue;

	SetRegister(A, result);
	SetFlag(ZeroFlag, result == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);
	SetFlag(CarryFlag, 0);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, A);
	}
}

void GameBoy::Xor(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t Acc = GetRegisterValue(A);
	uint8_t regValue = GetRegisterValue(reg);

	uint8_t result = Acc ^ regValue;

	SetRegister(A, result);
	SetFlag(ZeroFlag, result == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);
	SetFlag(CarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), A, reg);
	}
}

void GameBoy::ImmediateXor(uint8_t opcode)
{
	uint8_t immediateValue = ReadROMLine();
	uint8_t Acc = GetRegisterValue(A);
	
	uint8_t result = Acc ^ immediateValue;

	SetRegister(A, result);
	SetFlag(ZeroFlag, result == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);
	SetFlag(CarryFlag, 0);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, A);
	}
}

void GameBoy::ComplementCarryFlag(uint8_t opcode)
{
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);
	SetFlag(CarryFlag, !GetFlag(CarryFlag));
}

void GameBoy::SetCarryFlag(uint8_t opcode)
{
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);
	SetFlag(CarryFlag, 1);
}

void GameBoy::ComplementAccumulator(uint8_t opcode)
{
	SetRegister(A, ~GetRegisterValue(A));
	SetFlag(NegativeFlag, 1);
	SetFlag(HalfCarryFlag, 1);
}

void GameBoy::ImmediateLD(uint8_t opcode)
{
	uint8_t reg1 = (opcode & 0b00111000) >> 3;
	uint8_t immediateValue = ReadROMLine();
	SetRegister(reg1, immediateValue);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, reg1);
	}
}

void GameBoy::Jump(uint8_t opcode)
{
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	SetProgramCounter(address);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), address);
	}
}

void GameBoy::JumpToHL(uint8_t opcode)
{
	SetProgramCounter(Get16BitRegisterValue(HL16));
	
	if (printLogs)
	{
		std::cout << InstructionToString(DecodeOpcode(opcode)) << "\n";
	}
}

void GameBoy::ConditionalJump(uint8_t opcode)
{
	uint8_t condition = (opcode & 0b00011000) >> 3;
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t jumpAddress = (uint16_t(msb) << 8) + lsb;
	
	if (CheckCondition(condition))
	{
		SetProgramCounter(jumpAddress);
	}

	if (printLogs)
	{
		std::cout << ConditionToString(condition) << " ";
		PrintImmediateInstruction(DecodeOpcode(opcode), jumpAddress);
	}
}

void GameBoy::RelativeJump(uint8_t opcode)
{
	int8_t offset = ReadROMLine();
	IncrementProgramCounter(offset);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), programCounter);
	}
}

void GameBoy::ConditionalRelativeJump(uint8_t opcode)
{
	uint8_t condition = (opcode & 0b00011000) >> 3;
	int8_t offset = ReadROMLine();
	uint16_t oldProgramCounter = programCounter;

	if (CheckCondition(condition))
	{
		IncrementProgramCounter(offset);
	}

	if (printLogs)
	{
		std::cout << ConditionToString(condition) << " ";
		PrintImmediateInstruction(DecodeOpcode(opcode), oldProgramCounter + offset);
	}
}
void GameBoy::FunctionCall(uint8_t opcode)
{
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	uint8_t current_msb = GetHighByte(programCounter);
	uint8_t current_lsb = GetLowByte(programCounter);
	Push(current_msb);
	Push(current_lsb);
	SetProgramCounter(address);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), address);
	}
}

void GameBoy::ConditionalFunctionCall(uint8_t opcode)
{
	uint8_t condition = (opcode & 0b00011000) >> 3;
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	uint8_t current_msb = GetHighByte(programCounter);
	uint8_t current_lsb = GetLowByte(programCounter);

	if (CheckCondition(condition))
	{
		Push(current_msb);
		Push(current_lsb);
		SetProgramCounter(address);
	}

	if (printLogs)
	{
		std::cout << ConditionToString(condition) << " ";
		PrintImmediateInstruction(DecodeOpcode(opcode), address);
	}
}

void GameBoy::FunctionReturn(uint8_t opcode)
{
	SetProgramCounter(Pop() + (Pop() << 8));

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), programCounter);
	}
}

void GameBoy::ConditionalFunctionReturn(uint8_t opcode)
{
	uint8_t condition = (opcode & 0b00011000) >> 3;
	if (printLogs)
	{
		std::cout << ConditionToString(condition) << " ";
	}

	if (CheckCondition(condition))
	{
		FunctionReturn(opcode);
	}

	else if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), programCounter);
	}
}

void GameBoy::CPL(uint8_t opcode)
{
	SetRegister(A, ~GetRegisterValue(A));
	SetFlag(NegativeFlag, true);
	SetFlag(HalfCarryFlag, true);
	if (printLogs)
	{
		std::cout << "CPL\n";
	}
}

void GameBoy::SCF(uint8_t opcode)
{
	SetFlag(CarryFlag, true);
	SetFlag(NegativeFlag, false);
	SetFlag(HalfCarryFlag, false);
}

void GameBoy::CCF(uint8_t opcode)
{
	SetFlag(CarryFlag, !GetFlag(CarryFlag));
	SetFlag(NegativeFlag, false);
	SetFlag(HalfCarryFlag, false);
}



void GameBoy::InterruptReturn(uint8_t opcode)
{
	IME = 1;
	FunctionReturn(opcode);

	UpdateClock(12);
	
	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void GameBoy::Restart(uint8_t opcode)
{
	uint16_t address = ((opcode & 0b00111000) >> 3) * 8;
	uint8_t current_msb = GetHighByte(programCounter);
	uint8_t current_lsb = GetLowByte(programCounter);
	Push(current_msb);
	Push(current_lsb);
	SetProgramCounter(address);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void GameBoy::Halt(uint8_t opcode)
{
	haltMode = true;
	UpdateClock(4);
	
	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void GameBoy::Stop(uint8_t opcode)
{
	systemClockActive = 0;
	mainClockActive = 0;
	memory[0xFF04] = 0;
	
	if (printLogs)
	{
		std::cout << "Stop\n";
	}
}

void GameBoy::DisableInterrupts(uint8_t opcode)
{
	IME = 0;

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void GameBoy::EnableInterrupts(uint8_t opcode)
{
	IME = 1;

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void GameBoy::Bit(uint8_t opcode)
{
	uint8_t bit = (opcode - 0x40) / 8;
	uint8_t reg = (opcode % 0x08);
	bool result = ExtractBit(GetRegisterValue(reg), bit);

	SetFlag(ZeroFlag, !result);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 1);

	UpdateClock(4);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeCBOpcode(opcode), bit, reg);
	}
}

void GameBoy::SLA(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = oldValue << 1;
	SetRegister(reg, newValue);

	SetFlag(CarryFlag, ExtractBit(oldValue, 7));
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	UpdateClock(4);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::SRA(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = (oldValue >> 1) + (oldValue & (1u << 7)); // MSB needs to stay the same

	SetRegister(reg, newValue);
	SetFlag(CarryFlag, ExtractBit(oldValue, 0));
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	UpdateClock(4);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::SET(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t bit = (opcode - 0xC0) / 8;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = oldValue | (1 << bit);
	SetRegister(reg, newValue);
	UpdateClock(4);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::RES(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t bit = (opcode - 0x80) / 8;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = oldValue & (~(1 << bit));
	SetRegister(reg, newValue);
	UpdateClock(4);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::SWAP(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t topNibble = oldValue >> 4;
	uint8_t bottomNibble = oldValue & 0b00001111;
	uint8_t newValue = (bottomNibble << 4) + topNibble;

	SetRegister(reg, newValue);
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(CarryFlag, 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	UpdateClock(4);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void GameBoy::Nop(uint8_t opcode)
{
	UpdateClock(4);

	if (printLogs)
	{
		std::cout << "Nop\n";
	}
}

void GameBoy::FIFO::FetchPixels()
{
	// Get tile
	uint8_t LCDControl = gameboy.memory[0xFF40];
	uint8_t LY = gameboy.memory[0xFF44];
	uint16_t tileMapAddress = 0x9800;

	// todo: and scanline x in window
	if (ExtractBit(LCDControl, 3))
	{
		tileMapAddress = 0x9C00;
	}

	// todo: and scanline x in window
	else if (ExtractBit(LCDControl, 6))
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
	uint8_t LCDControl = memory[0xFF40];
	uint8_t LY = memory[0xFF44];
	bool LCDOn = ExtractBit(LCDControl, 7);
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
	uint64_t previousScanlineNumber = ppu_cycles / 456;
	uint64_t nextScanlineNumber = (ppu_cycles + offset) / 456;
	uint8_t LY = memory[0xFF44] + (nextScanlineNumber - previousScanlineNumber);

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


// Update the gameboy's clock and everything that depends on it
void GameBoy::UpdateClock(uint32_t cycleCount)
{
	// todo: other stuff that needs updating
	UpdatePPU(cycleCount);
	cpu_cycles += cycleCount;
	ppu_cycles += cycleCount;

	if (DMA_transfer && cpu_cycles - DMA_start > 160)
	{
		DMA_transfer = false;
		for (uint16_t offset = 0; offset <  160; offset++)
		{
			memory[0xFE00 + offset] = memory[DMA_address + offset];
		}
	}

	uint8_t& divider_register = memory[0xFF04];
	uint8_t& timer_counter = memory[0xFF05];
	uint8_t& timer_modulo = memory[0xFF06];
	uint8_t& timer_control = memory[0xFF07];
	uint8_t& interrupt_requests = memory[0xFF0F];

	if (systemClockActive)
	{
		div_clock += cycleCount;
		if (div_clock > 256)
		{
			div_clock -= 256;
			divider_register++;
		}
	}

	auto increment_timer = [&]() {
		if (timer_counter == 0xFF)
		{
			interrupt_requests |= (1 << 2);
			timer_counter = timer_modulo;
		}
		else
		{
			timer_counter++;
		}
	};

	if (ExtractBit(timer_control, 2))
	{
		timer_clock += cycleCount;
		switch (timer_control & 0b11)
		{
		case 0:
			if (timer_clock > 1024)
			{
				timer_clock -= 1024;
				increment_timer();
			}
		case 1:
			if (timer_clock > 16)
			{
				timer_clock -= 16;
				increment_timer();
			}
		case 2:
			if (timer_clock > 64)
			{
				timer_clock -= 64;
				increment_timer();
			}
		case 3:
			if (timer_clock > 256)
			{
				timer_clock -= 256;
				increment_timer();
			}
		}
	}
}

void GameBoy::UpdateJoypadInput()
{
	uint8_t& joypad = memory[0xFF00];

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

	if (!ExtractBit(joypad, 4))
	{
		setBit(0, !right_pressed);
		setBit(1, !left_pressed);
		setBit(2, !up_pressed);
		setBit(3, !down_pressed);
	}

	else if (!ExtractBit(joypad, 5))
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
}

std::string GameBoy::RegisterToString(uint8_t reg)
{
	switch (reg)
	{
	case B:
		return "B";
	case C:
		return "C";
	case D:
		return "D";
	case E:
		return "E";
	case H:
		return "H";
	case L:
		return "L";
	case HL:
		return "HL";
	case A:
		return "A";
	default:
		_ASSERT(false);
	}
}

std::string GameBoy::RegisterToString16(uint8_t reg)
{
	switch (reg)
	{
	case BC16:
		return "BC";
	case DE16:
		return "DE";
	case HL16:
		return "HL";
	case SP16:
		return "SP";
	default:
		_ASSERT(false);
	}
}

std::string GameBoy::InstructionToString(Instruction instruction)
{
	switch (instruction)
	{
	case Instruction::NOP:
		return "Nop";
	case Instruction::STOP:
		return "Stop";
	case Instruction::JR:
		return "JR";
	case Instruction::JR_COND:
		return "JR_COND";
	case Instruction::LD:
		return "LD";
	case Instruction::LD_IMMEDIATE:
		return "LD";
	case Instruction::LD_16:
		return "LD16";
	case Instruction::LD_16_IMMEDIATE:
		return "LDI16";
	case Instruction::LD_INDIRECT:
		return "LDI";
	case Instruction::LD_TO_INDIRECT:
		return "LDH";
	case Instruction::LD_TO_ACC:
		return "LDA";
	case Instruction::LD_FROM_ACC:
		return "LDFromA";
	case Instruction::LD_TO_ADDRESS:
		return "LDtoAddr";
	case Instruction::LD_FROM_ADDRESS:
		return "LDfromAddr";
	case Instruction::INC:
		return "INC";
	case Instruction::INC_16:
		return "INC16";
	case Instruction::DEC:
		return "DEC";
	case Instruction::DEC_16:
		return "DEC16";
	case Instruction::RLCA:
		return "RLCA";
	case Instruction::RLA:
		return "RLA";
	case Instruction::DAA:
		return "DAA";
	case Instruction::SCF:
		return "SCF";
	case Instruction::LD_A16:
		return "LD_A16";
	case Instruction::LD_TO_16:
		return "LD_TO_16";
	case Instruction::RRCA:
		return "RRCA";
	case Instruction::RRA:
		return "RRA";
	case Instruction::CPL:
		return "CPL";
	case Instruction::CCF:
		return "CCF";
	case Instruction::HALT:
		return "HALT";
	case Instruction::ADD:
		return "ADD";
	case Instruction::ADD_IMMEDIATE:
		return "ADDI";
	case Instruction::ADD_16:
		return "ADD16";
	case Instruction::ADC:
		return "ADC";
	case Instruction::ADC_IMMEDIATE:
		return "ADCI";
	case Instruction::SUB:
		return "SUB";
	case Instruction::SUB_IMMEDIATE:
		return "SUBI";
	case Instruction::SBC:
		return "SBC";
	case Instruction::SBC_IMMEDIATE:
		return "SBCI";
	case Instruction::AND:
		return "AND";
	case Instruction::AND_IMMEDIATE:
		return "ANDI";
	case Instruction::XOR:
		return "XOR";
	case Instruction::XOR_IMMEDIATE:
		return "XORI";
	case Instruction::OR:
		return "OR";
	case Instruction::OR_IMMEDIATE:
		return "ORI";
	case Instruction::CP:
		return "CP";
	case Instruction::CP_IMMEDIATE:
		return "CPI";
	case Instruction::RET:
		return "RET";
	case Instruction::RET_COND:
		return "RET_COND";
	case Instruction::LDH:
		return "LDH";
	case Instruction::POP_16:
		return "POP16";
	case Instruction::JP:
		return "JP";
	case Instruction::JP_HL:
		return "JP_HL";
	case Instruction::JP_COND:
		return "JP_COND";
	case Instruction::CALL:
		return "CALL";
	case Instruction::CALL_COND:
		return "CALL_COND";
	case Instruction::PUSH_16:
		return "PUSH16";
	case Instruction::RST:
		return "RST";
	case Instruction::RETI:
		return "RETI";
	case Instruction::EI:
		return "EI";
	case Instruction::DI:
		return "DI";
	case Instruction::PREFIX_CB:
		return "PREFIX_CB";
	case Instruction::RLC:
		return "RLC";
	case Instruction::RRC:
		return "RRC";
	case Instruction::RL:
		return "RL";
	case Instruction::RR:
		return "RR";
	case Instruction::SLA:
		return "SLA";
	case Instruction::SRA:
		return "SRA";
	case Instruction::SWAP:
		return "SWAP";
	case Instruction::SRL:
		return "SRL";
	case Instruction::BIT:
		return "BIT";
	case Instruction::RES:
		return "RES";
	case Instruction::SET:
		return "SET";
	default:
		_ASSERT(false);
	}
}

void GameBoy::PrintInstruction(Instruction instruction, uint8_t reg1, uint8_t reg2)
{
	std::string instruction_string = InstructionToString(instruction);
	std::string reg1_string = reg1 == 0xFF ? "" : RegisterToString(reg1);
	std::string reg2_string = reg2 == 0xFF ? "" : ", " + RegisterToString(reg2);

	std::cout << instruction_string << " " << reg1_string << reg2_string << "\n";
}

void GameBoy::PrintInstruction16(Instruction instruction, uint8_t reg1, uint8_t reg2)
{
	std::string instruction_string = InstructionToString(instruction);
	std::string reg1_string = reg1 == 0xFF ? "" : RegisterToString16(reg1);
	std::string reg2_string = reg2 == 0xFF ? "" : ", " + RegisterToString16(reg2);

	std::cout << instruction_string << " " << reg1_string << reg2_string << "\n";
}

void GameBoy::PrintImmediateInstruction(Instruction instruction, uint16_t immediateValue, uint8_t reg1)
{
	std::string instruction_string = InstructionToString(instruction);
	std::string reg1_string = reg1 == 0xFF ? "" : RegisterToString(reg1) + ", ";

	std::cout << instruction_string << " " << reg1_string << std::format("${:X}", immediateValue) << "\n";
}

void GameBoy::PrintImmediateInstructionReversed(Instruction instruction, uint16_t immediateValue, uint8_t reg1)
{
	std::string instruction_string = InstructionToString(instruction);
	std::string reg1_string = reg1 == 0xFF ? "" : RegisterToString(reg1);

	std::cout << instruction_string << " " << std::format("${:X}", immediateValue) << ", " << reg1_string << "\n";
}

std::string GameBoy::ConditionToString(uint8_t condition)
{
	switch (condition)
	{
	case 0:
		return "[NZ]";
	case 1:
		return "[Z]";
	case 2:
		return "[NC]";
	case 3:
		return "[C]";
	default:
		_ASSERT(false);
	}
}

void GameBoy::LogCPUState()
{
	cpu_state << std::format("A:{:02X} F:{:02X} B:{:02X} C:{:02X} D:{:02X} E:{:02X} H:{:02X} L:{:02X} SP:{:04X} PC:{:04X} PCMEM:{:02X},{:02X},{:02X},{:02X}",
		registerA, registerF, registerB, registerC, registerD, registerE, registerH, registerL, stackPointer, programCounter,
		Read(programCounter), Read(programCounter + 1), Read(programCounter + 2), Read(programCounter + 3)) << std::endl;

}

