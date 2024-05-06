#include "gameboy.hpp"
#include"memory.hpp"
#include"joypad.hpp"
#include"apu.hpp"
#include"cpu.hpp"
#include"ppu.hpp"
#include "helpers.hpp"


GameBoy::GameBoy() 
{
	memory = new Memory();
	cpu = new CPU();
	joypad = new Joypad(memory, cpu);
	ppu = new PPU(memory, cpu);
	apu = new APU(memory);
	memory->SetPointers(cpu, apu, joypad, this);
	cpu->SetPointers(memory, apu, ppu, this);
}

GameBoy::~GameBoy()
{
	delete memory;
	delete cpu;
	delete ppu;
	delete apu;
	delete joypad;
}

void GameBoy::Reset()
{
	ppu->Init();
	apu->InitAPU();
}

void GameBoy::SetToPostBootState()
{
	Reset();
	cpu->Init();
	memory->SetToPostBootState();
	ppu->clock = 0;
}

void GameBoy::ExecutionLoop()
{
	while (true)
	{
		/*if (cpu_cycles == 494400) { __debugbreak(); }*/
		if (!cpu->haltMode && cpu->printLogs)
		{
			cpu->LogCPUStateDetailed();
		}

		if (cpu->HandleInterrupts())
		{
			cpu->LogCPUStateDetailed();
		}

		uint16_t previousPC = cpu->programCounter;
		if (!cpu->haltMode)
		{
			uint8_t opcode = cpu->ReadROMLine();
			cpu->ExecuteInstruction(opcode);
		}
		else
		{
			UpdateClock(4);
		}
	

		if (ppu->clock >= 70224)
		{
			if (Helpers::ExtractBit(memory->mainMemory[0xFF40], 7) && Helpers::ExtractBit(memory->mainMemory[0xFF40], 0))
			{
				ppu->DrawScreen();
				WaitForFrameTime();
				ppu->clock %= 70224;
			}		
		}
	}

	ppu->Terminate();
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
	memory->LoadRom(filename, startAddress);
}

void GameBoy::LoadCartridge(std::string filename)
{
	memory->LoadCartridge(filename);
}

void GameBoy::UpdateClock(uint32_t cycleCount)
{
	ppu->Update(cycleCount);
	apu->UpdateAPUTimers(cycleCount);
	cpu->UpdateClock(cycleCount);
	memory->UpdateClock(cycleCount);
}
