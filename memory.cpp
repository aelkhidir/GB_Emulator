#include "memory.hpp"
#include "cpu.hpp"
#include "apu.hpp"
#include "joypad.hpp"
#include "gameboy.hpp"
#include "helpers.hpp"

void Memory::SetPointers(CPU* cpu_pointer, APU* apu_pointer, Joypad* joypad_pointer, GameBoy* gameboy_pointer)
{
	cpu = cpu_pointer;
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
	mainMemory[0xFF00] = 0xCF;
	mainMemory[0xFF01] = 0x00;
	mainMemory[0xFF02] = 0x7E;
	mainMemory[0xFF04] = 0xAB;
	mainMemory[0xFF05] = 0x00;
	mainMemory[0xFF06] = 0x00;
	mainMemory[0xFF07] = 0xF8;
	mainMemory[0xFF0F] = 0xE1;
	mainMemory[0xFF10] = 0x80;
	mainMemory[0xFF11] = 0xBF;
	mainMemory[0xFF12] = 0xF3;
	mainMemory[0xFF13] = 0xFF;
	mainMemory[0xFF14] = 0xBF;
	mainMemory[0xFF16] = 0x3F;
	mainMemory[0xFF17] = 0x00;
	mainMemory[0xFF18] = 0xFF;
	mainMemory[0xFF19] = 0xBF;
	mainMemory[0xFF1A] = 0x7F;
	mainMemory[0xFF1B] = 0xFF;
	mainMemory[0xFF1C] = 0x9F;
	mainMemory[0xFF1D] = 0xFF;
	mainMemory[0xFF1E] = 0xBF;
	mainMemory[0xFF20] = 0xFF;
	mainMemory[0xFF21] = 0x00;
	mainMemory[0xFF22] = 0x00;
	mainMemory[0xFF23] = 0xBF;
	mainMemory[0xFF24] = 0x77;
	mainMemory[0xFF25] = 0xF3;
	mainMemory[0xFF26] = 0xF1;
	mainMemory[0xFF40] = 0x91;
	mainMemory[0xFF41] = 0x85;
	mainMemory[0xFF42] = 0x00;
	mainMemory[0xFF43] = 0x00;
	mainMemory[0xFF44] = 0x00;
	mainMemory[0xFF45] = 0x00;
	mainMemory[0xFF46] = 0xFF;
	mainMemory[0xFF47] = 0xFC;

	// These aren't initialised in the boot rom, but it's better to keep them stable
	mainMemory[0xFF48] = 0x00;
	mainMemory[0xFF49] = 0x00;

	mainMemory[0xFF4A] = 0x00;
	mainMemory[0xFF4B] = 0x00;
	mainMemory[0xFF4D] = 0xFF;
	mainMemory[0xFF4F] = 0xFF;
	mainMemory[0xFF51] = 0xFF;
	mainMemory[0xFF52] = 0xFF;
	mainMemory[0xFF53] = 0xFF;
	mainMemory[0xFF54] = 0xFF;
	mainMemory[0xFF55] = 0xFF;
	mainMemory[0xFF56] = 0xFF;
	mainMemory[0xFF68] = 0xFF;
	mainMemory[0xFF69] = 0xFF;
	mainMemory[0xFF6A] = 0xFF;
	mainMemory[0xFF6B] = 0xFF;
	mainMemory[0xFF70] = 0xFF;
	mainMemory[0xFFFF] = 0x00;


	/*if (printLogs)
	{
		LogMemoryState();
	}*/
}

void Memory::UpdateClock(uint64_t cycles)
{
	mem_clock += cycles;
}

uint8_t Memory::Read(uint16_t address)
{
	gameboy->UpdateClock(4);

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
		joypad->UpdateJoypadInput();
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

void Memory::Write(uint8_t value, uint16_t address)
{
	gameboy->UpdateClock(4);

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
		serial_transfer << char(mainMemory[0xFF01]);
		serial_transfer.flush();
		mainMemory[0xFF02] = 0;
		return;
	}

	// divider register resets when written
	if (address == 0xFF04)
	{
		mainMemory[0xFF04] = 0;
		cpu->div_clock = 0;
		return;
	}

	// OAM DMA transfer
	if (address == 0xFF46 && !DMA_transfer)
	{
		uint16_t source_address = value * 0x100;
		cpu->StartDMATransfer(source_address);
	}

	// Trigger audio channel 1
	if (address == 0xFF14 && Helpers::ExtractBit(value, 7))
	{
		apu->TriggerChannel1();
	}

	if (address == 0xFF19 && Helpers::ExtractBit(value, 7))
	{
		apu->TriggerChannel2();
	}

	// Trigger audio channel 3
	if (address == 0xFF1E && Helpers::ExtractBit(value, 7))
	{
		apu->TriggerChannel3();
	}

	if (address == 0xFF1A)
	{
		apu->ControlChannel3(value);
	}

	if (address >= 0x8000 && address < 0xA000 && CanAccessVRAM())
	{
		memory_state << std::format("Writing {:02X} to address {:04X} in VRAM\n", value, address);
	}

	mainMemory[address] = value;
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
	return mainMemory[address]; // Todo: access checks first
}

void Memory::Set(uint16_t address, uint8_t value)
{
	mainMemory[address] = value; // Todo: maybe specialise into (SetJoypad, Set...)
}

void Memory::LogMemoryState()
{
	for (uint32_t i = 0; i < 0x10000; i++)
	{
		memory_state << unsigned(mainMemory[i]) << std::endl;
	}
}