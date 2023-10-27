#pragma once
#define SDL_MAIN_HANDLED
#include <cassert>
#include<iostream>
#include<sstream>
#include<fstream>
#include<format>
#include<vector>
#include<string>
#include<array>
#include<map>
#include <queue>
#include<Windows.h>
#include <SDL.h>
#include <cstdio>

constexpr auto B = 0;
constexpr auto C = 1;
constexpr auto D = 2;
constexpr auto E = 3;
constexpr auto H = 4;
constexpr auto L = 5;
constexpr auto HL = 6;
constexpr auto A = 7;
constexpr auto BC16 = 0;
constexpr auto DE16 = 1;
constexpr auto HL16 = 2;
constexpr auto SP16 = 3;
constexpr auto ZeroFlag = 7;
constexpr auto NegativeFlag = 6;
constexpr auto HalfCarryFlag = 5;
constexpr auto CarryFlag = 4;

enum class Instruction;

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
		serial_transfer("serial_transfer.txt"),
		cpu_state("cpu_state.txt")
	{}
	
	~GameBoy()
	{
		delete[] memory;
	}
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

	// Main stuff
	uint8_t registerA, registerB, registerC, registerD, registerE, registerF, registerH, registerL;
	uint16_t programCounter = 0;
	uint16_t stackPointer = 0;
	uint8_t* memory = new uint8_t[0x10000]();
	uint8_t cartridgeMemory[0x100];

	// Interrupts, clock
	bool IME = 0;
	bool ScheduledIME = 0;
	bool systemClockActive = 1;
	bool mainClockActive = 1;
	bool haltMode = false;
	bool haltBug = false;
	uint64_t cycles = 0;

	// real clock for emulating FPS
	clock_t realTime = clock();

	// Screen
	SDL_Event event;
	SDL_Renderer* renderer;
	SDL_Window* window;

	// Pixel FIFO
	FIFO backgroundFifo = FIFO(*this);
	FIFO spriteFifo = FIFO(*this);

	// Options 
	bool printLogs = true;

	// Serial Transfer Stream
	std::ofstream serial_transfer;

	// Other logging stuff
	std::ofstream cpu_state;

public:
	void Reset();

	void SetToPostBootState();

	void ExecutionLoop();

	void WaitForFrameTime();

	void HandleInterrupts();

	void ResetInterruptFlag(uint8_t num);

	void PointToCartridgeMemory();

	void Write(uint8_t value, uint16_t address);

	uint8_t Read(uint16_t address);

	void UpdateVBlank();

	void WriteScanlineY(uint8_t value);

	void UpdatePPU(uint64_t cycleCount);

	bool CanAccessVRAM();

	std::map<std::pair<uint8_t, uint8_t>, uint8_t> DecodeTileMap();

	std::array<uint8_t, 64> ExtractTileData(uint8_t offset);

	uint8_t GetColor(uint8_t index);

	void DrawScreen();

	void RenderTileMap();

	uint8_t CheckCondition(uint8_t condition);

	Instruction DecodeOpcode(uint8_t opcode);

	Instruction DecodeCBOpcode(uint8_t opcode);

	void IncrementProgramCounter(int8_t offset);

	void SetProgramCounter(uint16_t value);

	void ExecuteCBInstruction();

	void ImmediateOr(uint8_t opcode);

	void Xor(uint8_t opcode);

	void Bit(uint8_t opcode);

	void ImmediateXor(uint8_t opcode);

	void ComplementCarryFlag(uint8_t opcode);

	void SetCarryFlag(uint8_t opcode);

	void ComplementAccumulator(uint8_t opcode);

	void ImmediateLD(uint8_t opcode);

	void ImmediateLD16(uint8_t opcode);

	void IndirectLD(uint8_t opcode);

	void LDAccumulator(uint8_t opcode);

	void LDfromAccumulator(uint8_t opcode);

	void LDtoAddress(uint8_t opcode);

	void LDStackPointertoAddress(uint8_t opcode);
	
	void LDfromAddress(uint8_t opcode);

	void LDStackPointerFromHL(uint8_t opcode);

	void LDHLFromStackPointer(uint8_t opcode);

	void Jump(uint8_t opcode);

	void JumpToHL(uint8_t opcode);

	void ConditionalJump(uint8_t opcode);

	void RelativeJump(uint8_t opcode);

	void ConditionalRelativeJump(uint8_t opcode);

	void FunctionCall(uint8_t opcode);

	void ConditionalFunctionCall(uint8_t opcode);

	void FunctionReturn(uint8_t opcode);

	void ConditionalFunctionReturn(uint8_t opcode);

	void CPL(uint8_t opcode);

	void InterruptReturn(uint8_t opcode);

	void Restart(uint8_t opcode);

	void Halt(uint8_t opcode);

	void Stop(uint8_t opcode);

	void DisableInterrupts(uint8_t opcode);

	void EnableInterrupts(uint8_t opcode);

	void Nop(uint8_t opcode);

	Mode CalculatePPUMode(uint64_t offset=0);

	void UpdateClock(uint32_t cycleCount);

	std::string RegisterToString(uint8_t reg);

	std::string RegisterToString16(uint8_t reg);

	std::string InstructionToString(Instruction instruction);

	void PrintInstruction(Instruction instruction, uint8_t reg1=0xFF, uint8_t reg2=0xFF);

	void PrintInstruction16(Instruction instruction, uint8_t reg1=0xFF, uint8_t reg2=0xFF);

	void PrintImmediateInstruction(Instruction instruction, uint16_t immediateValue, uint8_t reg1=0xFF);

	void PrintImmediateInstructionReversed(Instruction instruction, uint16_t immediateValue, uint8_t reg1=0xFF);

	std::string ConditionToString(uint8_t condition);

	uint8_t ReadROMLine();

	void Push(uint8_t value);

	void Push16(uint8_t opcode);

	uint8_t Pop();
	
	void Pop16(uint8_t opcode);

	void ExecuteInstruction(uint8_t opcode);

	void LD(uint8_t opcode);

	void LDH(uint8_t opcode);

	void LDtoIndirect(uint8_t opcode);

	void Add(uint8_t opcode);

	void Add16(uint8_t opcode);

	void ImmediateAdd(uint8_t opcode);

	void ImmediateAddToStackPointer(uint8_t opcode);

	void Adc(uint8_t opcode);

	void ImmediateAdc(uint8_t opcode);

	uint8_t GetRegisterValue(uint8_t num);

	uint16_t Get16BitRegisterValue(uint8_t num);

	void Set16BitRegister(uint8_t num, uint16_t value);

	void SetRegister(uint8_t num, uint16_t value);

	void SetFlag(uint8_t num, bool value);

	bool GetFlag(uint8_t num);

	void Sub(uint8_t opcode);

	void ImmediateSub(uint8_t opcode);

	void SBC(uint8_t opcode);

	void Compare(uint8_t opcode);

	void ImmediateCompare(uint8_t opcode);

	void RotateLeftAccumulator(uint8_t opcode);

	void RotateRightAccumulator(uint8_t opcode);

	void RotateLeft(uint8_t opcode);

	void RotateRight(uint8_t opcode);

	void RotateLeftCircular(uint8_t opcode);

	void RotateRightCircular(uint8_t opcode);

	void ShiftRightThroughCarry(uint8_t opcode);

	void RotateLeftThroughCarry(uint8_t reg);

	void RotateRightThroughCarry(uint8_t reg);

	void Increment(uint8_t opcode);

	void Increment16(uint8_t opcode);

	void Decrement(uint8_t opcode);

	void Decrement16(uint8_t opcode);

	void And(uint8_t opcode);

	void ImmediateAnd(uint8_t opcode);

	void Or(uint8_t opcode);

	void SCF(uint8_t opcode);

	void CCF(uint8_t opcode);

	void RLCA(uint8_t opcode);

	void RRCA(uint8_t opcode);

	void SLA(uint8_t opcode);

	void SRA(uint8_t opcode);

	void SWAP(uint8_t opcode);

	void RES(uint8_t opcode);

	void SET(uint8_t opcode);

	
	void LoadRom(std::string filename, uint16_t startAddress);

	void LoadCartridge(std::string filename);

	void UnmapBootRom();

	void DrawFullBackground();

	void DrawTileMap();

	std::map<std::pair<uint8_t, uint8_t>, uint8_t> ExtractFullTileMap();

	bool CheckPPUtoVRAMAccess();

	void LogCPUState();
};


enum class Instruction
{
	NOP,
	STOP,
	JR,
	JR_COND,
	LD,
	LD_IMMEDIATE,
	LD_16,
	LD_16_IMMEDIATE,
	LD_INDIRECT,
	LD_TO_INDIRECT,
	LD_TO_ACC,
	LD_FROM_ACC,
	LD_TO_ADDRESS,
	LD_FROM_ADDRESS,
	LD_SP_FROM_HL,
	LD_HL_FROM_SP,
	INC,
	INC_16,
	DEC,
	DEC_16,
	RLCA,
	RLA,
	DAA,
	SCF,
	LD_A16,
	LD_TO_16,
	RRCA,
	RRA,
	CPL,
	CCF,
	HALT,
	ADD,
	ADD_IMMEDIATE,
	ADD_16,
	ADD_SP_IMMEDIATE,
	ADC,
	ADC_IMMEDIATE,
	SUB,
	SUB_IMMEDIATE,
	SBC,
	SBC_IMMEDIATE,
	AND,
	AND_IMMEDIATE,
	XOR,
	XOR_IMMEDIATE,
	OR,
	OR_IMMEDIATE,
	CP,
	CP_IMMEDIATE,
	RET,
	RET_COND,
	LDH,
	POP_16,
	JP,
	JP_HL,
	JP_COND,
	CALL,
	CALL_COND,
	PUSH_16,
	RST,
	RETI,
	EI,
	DI,
	PREFIX_CB,
	RLC,
	RRC,
	RL,
	RR,
	SLA,
	SRA,
	SWAP,
	SRL,
	BIT,
	RES,
	SET,
};

enum class Mode
{
	Hblank = 0u,
	Vblank = 1u,
	OAMSearch = 2u,
	PixelTransfer = 3u
};