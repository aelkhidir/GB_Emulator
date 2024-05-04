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

class APU;
class CPU;
class Joypad;

enum class MBC
{
	ROM = 0,
	MBC1 = 1,
	MBC1_RAM = 2,
	MBC1_RAM_BATTERY = 3,
	MBC2 = 5,
	MBC2_BATTERY = 6,
	ROM_RAM = 8,
	ROM_RAM_BATTERY = 9
};

class Memory
{
public:
	Memory(CPU* cpu, APU* apu, Joypad* joypad) : 
		memory_state("memory_state.txt"), 
		serial_transfer("serial_transfer.txt"),
		cpu(cpu),
		apu(apu),
		joypad(joypad)
	{}
	~Memory()
	{
		delete[] mainMemory;
		delete[] cartridgeMemory;
		delete[] externalRAM;
	}

	uint8_t* mainMemory = new uint8_t[0x10000]{};
	uint8_t* cartridgeMemory = new uint8_t[0x100000]{};
	uint8_t* externalRAM = new uint8_t[0x4000]{};

	MBC mbcState = MBC::ROM;
	uint8_t bankBits = 5;
	uint8_t bankRegister = 0;
	uint8_t secondaryBankRegister = 0;
	uint8_t selectedBank = 1;
	bool bankMode = false;
	bool canAccessRam = false;
	bool DMA_transfer = false;

	std::ofstream serial_transfer;
	std::ofstream memory_state;
	bool printLogs = false;
	
	uint64_t mem_clock = 0;

	// Pointers needed to trigger events on memory read/write
	CPU* cpu;
	APU* apu;
	Joypad* joypad;

	void UnmapBootRom();
	void SetToPostBootState();
	void UpdateClock(uint64_t cycles);
	//void StartDMATransfer(uint16_t address);
	bool CanAccessVRAM();
	//bool CanAccessOAM();
	uint8_t Read(uint16_t address);
	uint8_t ReadMappedAddress(uint16_t address);
	void Write(uint8_t value, uint16_t address);
	uint8_t Get(uint16_t address); 
	void Set(uint16_t address, uint8_t value); // No clock penalty, used for updating the state internally
	void SelectMemoryBank();
	void LogMemoryState();

};