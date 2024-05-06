#include "memory.hpp"
#include "cpu.hpp"
#include "ppu.hpp"
#include "apu.hpp"
#include "joypad.hpp"
#include "gameboy.hpp"
#include "helpers.hpp"

void Memory::SetPointers(CPU* cpu_pointer, PPU* ppu_pointer, APU* apu_pointer, Joypad* joypad_pointer, GameBoy* gameboy_pointer)
{
	cpu = cpu_pointer;
	ppu = ppu_pointer;
	apu = apu_pointer;
	joypad = joypad_pointer;
	gameboy = gameboy_pointer;
}

void Memory::UnmapBootRom()
{
	for (uint32_t byteNumber = 0; byteNumber < 0x100; byteNumber++)
	{
		mainMemory[byteNumber] = cartridgeMemory[byteNumber];
	}
}

void Memory::LoadRom(std::string filename, uint16_t startAddress)
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
			mainMemory[startAddress + byteNumber] = bytes[byteNumber];
		}
		return;
	}
	_ASSERTE(false);
}

void Memory::LoadCartridge(std::string filename)
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
			cartridgeMemory[byteNumber] = bytes[byteNumber];
		}

		for (uint32_t byteNumber = 0x100; byteNumber < 0x8000; byteNumber++)
		{
			mainMemory[byteNumber] = cartridgeMemory[byteNumber];
		}

		mbcState = static_cast<MBC>(cartridgeMemory[0x147]);
		uint8_t memory_banks = bytes.size() / 0x4000;
		bankBits = std::bit_width(memory_banks) - 1; // log2
		return;
	}
	_ASSERTE(false);
}

void Memory::SetToPostBootState()
{
	bool unmapBootRom = true;

	if (unmapBootRom)
	{
		UnmapBootRom();
	}

	// hardware register values after the DMG boot rom
	Set(0xFF00, 0xCF);
	Set(0xFF01, 0x00);
	Set(0xFF02, 0x7E);
	Set(0xFF04, 0xAB);
	Set(0xFF05, 0x00);
	Set(0xFF06, 0x00);
	Set(0xFF07, 0xF8);
	Set(0xFF0F, 0xE1);
	Set(0xFF10, 0x80);
	Set(0xFF11, 0xBF);
	Set(0xFF12, 0xF3);
	Set(0xFF13, 0xFF);
	Set(0xFF14, 0xBF);
	Set(0xFF16, 0x3F);
	Set(0xFF17, 0x00);
	Set(0xFF18, 0xFF);
	Set(0xFF19, 0xBF);
	Set(0xFF1A, 0x7F);
	Set(0xFF1B, 0xFF);
	Set(0xFF1C, 0x9F);
	Set(0xFF1D, 0xFF);
	Set(0xFF1E, 0xBF);
	Set(0xFF20, 0xFF);
	Set(0xFF21, 0x00);
	Set(0xFF22, 0x00);
	Set(0xFF23, 0xBF);
	Set(0xFF24, 0x77);
	Set(0xFF25, 0xF3);
	Set(0xFF26, 0xF1);
	Set(0xFF40, 0x91);
	Set(0xFF41, 0x85);
	Set(0xFF42, 0x00);
	Set(0xFF43, 0x00);
	Set(0xFF44, 0x00);
	Set(0xFF45, 0x00);
	Set(0xFF46, 0xFF);
	Set(0xFF47, 0xFC);

	// These aren't initialised in the boot rom, but it's better to keep them stable
	Set(0xFF48, 0x00);
	Set(0xFF49, 0x00);

	Set(0xFF4A, 0x00);
	Set(0xFF4B, 0x00);
	Set(0xFF4D, 0xFF);
	Set(0xFF4F, 0xFF);
	Set(0xFF51, 0xFF);
	Set(0xFF52, 0xFF);
	Set(0xFF53, 0xFF);
	Set(0xFF54, 0xFF);
	Set(0xFF55, 0xFF);
	Set(0xFF56, 0xFF);
	Set(0xFF68, 0xFF);
	Set(0xFF69, 0xFF);
	Set(0xFF6A, 0xFF);
	Set(0xFF6B, 0xFF);
	Set(0xFF70, 0xFF);
	Set(0xFFFF, 0x00);
}

void Memory::UpdateClock(uint64_t cycles)
{
	mem_clock += cycles;
}

uint8_t Memory::Read(uint16_t address)
{
	gameboy->UpdateClock(4);

	if (address >= 0xFF00 && address <= 0xFF7F)
	{
		return ReadRegisterAddress(address);
	}

	if (address == 0xFFFF)
	{
		return cpu->interruptEnable;
	}

	if (address >= 0xFEA0 && address <= 0xFEFF)
	{
		return 0xFF;
	}

	if (DMA_transfer && !(address >= 0xFF80 && address <= 0xFFFE))
	{
		return 0x00;
	}

	if (mbcState == MBC::MBC1)
	{
		return ReadMappedAddress(address);
	}

	return mainMemory[address];
}

uint8_t Memory::ReadMappedAddress(uint16_t address)
{
	// Only MBC1 is implemented here
	uint8_t address_bank = 0;
	uint16_t mapped_address = 0;
	if (address <= 0x3FFF)
	{
		if (bankMode)
		{
			address_bank = secondaryBankRegister;
			mapped_address = 0x4000 * address_bank + (address % 0x4000);
			return cartridgeMemory[mapped_address];
		}
	}

	else if (address <= 0x7FFF)
	{
		address_bank = selectedBank;
		mapped_address = 0x4000 * address_bank + (address % 0x4000);
		return cartridgeMemory[mapped_address];
	}

	else if (address >= 0xA000 && address <= 0xBFFF)
	{
		address_bank = bankMode ? secondaryBankRegister : 0;

		// Check if RAM can be accessed and implement external RAM accesses
		if (false)
		{
			return 0x00;
		}

		if (address_bank == 0)
		{
			return mainMemory[address];
		}

		mapped_address = 0x2000 * (address_bank - 1) + (address % 0x2000);
		return externalRAM[mapped_address];
	}

	return mainMemory[address];
}

uint8_t Memory::ReadRegisterAddress(uint16_t address)
{
	_ASSERT(address >= 0xFF00 && address <= 0xFF7F);
	
	if (address >= 0xFF30 && address <= 0xFF3F)
	{
		// Wave RAM
		return mainMemory[address];
	}

	switch (address)
	{
	case 0xFF00: return joypad->GetJoypadInput();
	case 0xFF01: return mainMemory[0xFF01];
	case 0xFF02: return mainMemory[0xFF02];
	case 0xFF04: return cpu->dividerRegister;
	case 0xFF05: return cpu->timerCounter;
	case 0xFF06: return cpu->timerModulo;
	case 0xFF07: return cpu->timerControl;
	case 0xFF0F: return cpu->interruptRequests;
	case 0xFF10: return apu->ch1_sweep;
	case 0xFF11: return apu->ch1_length_clock;
	case 0xFF12: return apu->ch1_volume;
	case 0xFF13: return apu->ch1_period_low;
	case 0xFF14: return apu->ch1_period_high;
	case 0xFF16: return apu->ch2_length_clock;
	case 0xFF17: return apu->ch2_volume;
	case 0xFF18: return apu->ch1_period_low;
	case 0xFF19: return apu->ch2_period_high;
	case 0xFF1A: return apu->ch3_dac_enable;
	case 0xFF1B: return apu->ch3_length_clock;
	case 0xFF1C: return apu->ch3_output_level;
	case 0xFF1D: return apu->ch3_period_low;
	case 0xFF1E: return apu->ch3_period_high;
	case 0xFF20: return apu->ch4_timer;
	case 0xFF21: return apu->ch4_envelope;
	case 0xFF22: return apu->ch4_frequency;
	case 0xFF23: return apu->ch4_control;
	case 0xFF24: return apu->volume_and_vin_panning;
	case 0xFF25: return apu->panning;
	case 0xFF26: return apu->control;
	case 0xFF40: return ppu->LCDControl;
	case 0xFF41: return ppu->stat;
	case 0xFF42: return ppu->SCY;
	case 0xFF43: return ppu->SCX;
	case 0xFF44: return ppu->LY;
	case 0xFF45: return ppu->LYC;
	case 0xFF47: return ppu->BGPalette;
	case 0xFF48: return ppu->OBP0;
	case 0xFF49: return ppu->OBP1;
	case 0xFF50: return mainMemory[0xFF50];
	default:
		_ASSERT(false);
	}
}

void Memory::WriteRegisterAddress(uint8_t value, uint16_t address)
{
	_ASSERT(address >= 0xFF00 && address <= 0xFF7F);

	if (address >= 0xFF30 && address <= 0xFF3F)
	{
		// Wave RAM
		mainMemory[address] = value;
	}

	switch (address)
	{
	case 0xFF00: joypad->SetJoypadValue(value); return;
	case 0xFF01: mainMemory[0xFF01] = value; return;
	case 0xFF02: 
	{
		if (value == 0x81)
		{
			_ASSERT(serial_transfer.is_open());
			serial_transfer << char(mainMemory[0xFF01]);
			serial_transfer.flush();
			mainMemory[0xFF02] = 0;
			return;
		}
	}
	case 0xFF04: cpu->divClock = 0; return;
	case 0xFF05: cpu->timerCounter = value; return;
	case 0xFF06: cpu->timerModulo = value; return;
	case 0xFF07: cpu->timerControl = value; return;
	case 0xFF0f: cpu->interruptRequests = value; return;
	case 0xFF10: apu->ch1_sweep = value; return;
	case 0xFF11: apu->ch1_length_clock = value; return;
	case 0xFF12: apu->ch1_volume = value; return;
	case 0xFF13: apu->ch1_period_low = value; return;
	case 0xFF14: apu->ch1_period_high = value; return;
	case 0xFF16: apu->ch2_length_clock = value; return;
	case 0xFF17: apu->ch2_volume = value; return;
	case 0xFF18: apu->ch1_period_low = value; return;
	case 0xFF19: apu->ch2_period_high = value; return;
	case 0xFF1A: apu->ch3_dac_enable = value; return;
	case 0xFF1B: apu->ch3_length_clock = value; return;
	case 0xFF1C: apu->ch3_output_level = value; return;
	case 0xFF1D: apu->ch3_period_low = value; return;
	case 0xFF1E: apu->ch3_period_high = value; return;
	case 0xFF20: apu->ch4_timer = value; return;
	case 0xFF21: apu->ch4_envelope = value; return;
	case 0xFF22: apu->ch4_frequency = value; return;
	case 0xFF23: apu->ch4_control = value; return;
	case 0xFF24: apu->volume_and_vin_panning = value; return;
	case 0xFF25: apu->panning = value; return;
	case 0xFF26: apu->control = value; return;
	case 0xFF40: ppu->LCDControl = value; return;
	case 0xFF41: ppu->stat = value; return;
	case 0xFF42: ppu->SCY = value; return;
	case 0xFF43: ppu->SCX = value; return;
	case 0xFF44: ppu->LY = value; return;
	case 0xFF45: ppu->LYC = value; return;
	case 0xFF46: 
	{
		uint16_t source_address = value * 0x100;
		cpu->StartDMATransfer(source_address);
		return;
	}
	case 0xFF47: ppu->BGPalette = value; return;
	case 0xFF48: ppu->OBP0 = value; return;
	case 0xFF49: ppu->OBP1 = value; return;
	case 0xFF50: 
	{
		if (value != 0)
		{
			UnmapBootRom();
			return;
		}
	}
	default:
		mainMemory[address] = value; return;
	}
}

void Memory::Write(uint8_t value, uint16_t address)
{
	gameboy->UpdateClock(4);	

	if (address >= 0xFF00 && address <= 0xFF7F)
	{
		WriteRegisterAddress(value, address);
		return;
	}

	if (address == 0xFFFF)
	{
		cpu->interruptEnable = value;
		return;
	}

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

	mainMemory[address] = value;
}

void Memory::WriteMappedAddress(uint8_t value, uint16_t address)
{
	// MBC registers
	if (mbcState == MBC::MBC1)
	{
		if (address <= 0x1FFF && mbcState == MBC::MBC1)
		{
			// Only MBC1+RAM should be able to access RAM
			canAccessRam = ((value & 0x0f) == 0x0a);
			return;
		}

		if (address >= 0x2000 && address <= 0x3FFF)
		{
			bankRegister = value & 0x1F;
			bankRegister = (bankRegister == 0) ? 0x01 : bankRegister;
			SelectMemoryBank();
			return;
		}

		if (address >= 0x4000 && address <= 0x5FFF)
		{
			secondaryBankRegister = value & 0x03;
			SelectMemoryBank();
			return;
		}

		if (address >= 0x6000 && address <= 0x7FFF)
		{
			bankMode = value & 0x01;
			return;
		}

		if (address >= 0xA000 && address <= 0xBFFF)
		{
			uint8_t address_bank = bankMode ? secondaryBankRegister : 0;

			if (!canAccessRam)
			{
				return;
			}

			if (address_bank == 0)
			{
				mainMemory[address] = value;
				return;
			}

			uint8_t mapped_address = 0x2000 * (address_bank - 1) + (address % 0x2000);
			externalRAM[mapped_address] = value;
			return;
		}
	}
}

void Memory::SelectMemoryBank()
{
	if (bankBits <= 5)
	{
		switch (bankBits)
		{
		case 1:
			selectedBank = bankRegister & 0x01;
			break;
		case 2:
			selectedBank = bankRegister & 0x03;
			break;
		case 3:
			selectedBank = bankRegister & 0x07;
			break;
		case 4:
			selectedBank = bankRegister & 0x0f;
			break;
		case 5:
			selectedBank = bankRegister & 0x1f;
			break;
		default:
			assert(false, "Invalid number of bank bits!");
		}
	}

	else
	{
		uint8_t bank_select = (secondaryBankRegister << 5) + bankRegister;
		selectedBank = bank_select;
	}
}

bool Memory::CanAccessVRAM()
{
	uint8_t LCD_control = mainMemory[0xFF40];
	if (!Helpers::ExtractBit(LCD_control, 7))
	{
		return true;
	}
	uint8_t screenMode = mainMemory[0xFF41] & 0b00000011;

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

uint8_t Memory::Get(uint16_t address)
{
	if (address >= 0xFF00 && address <= 0xFF7F)
	{
		return ReadRegisterAddress(address);
	}

	if (address == 0xFFFF)
	{
		return cpu->interruptEnable;
	}

	return mainMemory[address]; // Todo: access checks first
}

void Memory::Set(uint16_t address, uint8_t value)
{
	if (address == 0xFFFF)
	{
		cpu->interruptEnable = value;
		return;
	}

	if (address >= 0xFF00 && address <= 0xFF7F)
	{
		WriteRegisterAddress(value, address);
		return;
	}

	mainMemory[address] = value; // Todo: maybe specialise into (SetJoypad, Set...)
}

void Memory::LogMemoryState()
{
	for (uint32_t i = 0; i < 0x10000; i++)
	{
		memory_state << unsigned(mainMemory[i]) << std::endl;
	}
}