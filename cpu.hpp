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

class Memory;
class APU;
class PPU;
class GameBoy;

class CPU
{
public:
	CPU(Memory* memory_pointer, APU* apu_pointer, PPU* ppu_pointer, GameBoy* gameboy_pointer) : memory(memory_pointer), apu(apu_pointer), ppu(ppu_pointer), gameboy(gameboy_pointer), cpu_state("cpu_state.txt") {}
	CPU() : cpu_state("cpu_state.txt") {};

	// Main stuff
	uint8_t registerA, registerB, registerC, registerD, registerE, registerF, registerH, registerL;
	uint16_t programCounter = 0;
	uint16_t stackPointer = 0;

	// Interrupts, clock
	bool IME = 0;
	bool systemClockActive = 1;
	bool mainClockActive = 1;
	uint16_t divClock = 0;
	bool previousTimerControl = false;
	uint8_t interruptEnable;
	uint8_t interruptRequests;

	// Timer stuff
	uint8_t dividerRegister;
	uint8_t timerCounter;
	uint8_t timerModulo;
	uint8_t timerControl;

	bool haltMode = false;
	bool haltBug = false;
	uint64_t cycles = 0;
	bool DMA_transfer = false;
	uint64_t DMA_start = 0;
	uint16_t DMA_address = 0;

	Memory* memory;
	APU* apu;
	PPU* ppu;
	GameBoy* gameboy;

	bool printLogs = false;

	std::ofstream cpu_state;

	void Init();
	void SetPointers(Memory* memory_pointer, APU* apu_pointer, PPU* ppu_pointer, GameBoy* gameboy_pointer);
	bool HandleInterrupts();
	void StartDMATransfer(uint16_t address);
	void RequestVblankInterrupt();
	void RequestStatInterrupt();
	void RequestTimerInterrupt();
	void RequestSerialInterrupt();
	void RequestJoypadInterrupt();
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
	void UpdateClock(uint32_t cycleCount);
	void ReadStateFromMemory();
	void WriteStateToMemory();
	std::string RegisterToString(uint8_t reg);
	std::string RegisterToString16(uint8_t reg);
	std::string InstructionToString(Instruction instruction);
	void PrintInstruction(Instruction instruction, uint8_t reg1 = 0xFF, uint8_t reg2 = 0xFF);
	void PrintInstruction16(Instruction instruction, uint8_t reg1 = 0xFF, uint8_t reg2 = 0xFF);
	void PrintImmediateInstruction(Instruction instruction, uint16_t immediateValue, uint8_t reg1 = 0xFF);
	void PrintImmediateInstruction16(Instruction instruction, uint16_t immediateValue, uint8_t reg1 = 0xFF);
	void PrintImmediateInstructionReversed(Instruction instruction, uint16_t immediateValue, uint8_t reg1 = 0xFF);
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
	void DAA(uint8_t opcode);
	void SBCImmediate(uint8_t opcode);
	void SLA(uint8_t opcode);
	void SRA(uint8_t opcode);
	void SWAP(uint8_t opcode);
	void RES(uint8_t opcode);
	void SET(uint8_t opcode);
	void LogCPUState();
	void LogCPUStateDetailed();
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
