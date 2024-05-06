#include "cpu.hpp"
#include "memory.hpp"
#include "joypad.hpp"
#include "apu.hpp"
#include "ppu.hpp"
#include "gameboy.hpp"
#include "helpers.hpp"


void CPU::SetPointers(Memory* memory_pointer, APU* apu_pointer, PPU* ppu_pointer, GameBoy* gameboy_pointer)
{
	memory = memory_pointer;
	apu = apu_pointer;
	ppu = ppu_pointer;
	gameboy = gameboy_pointer;
}

void CPU::Init()
{
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

	cycles = 0;
}

void CPU::RequestVblankInterrupt()
{
	interruptRequests |= (1 << 0);
}

void CPU::RequestStatInterrupt()
{
	interruptRequests |= (1 << 1);
}

void CPU::RequestTimerInterrupt()
{
	interruptRequests |= (1 << 2);
}

void CPU::RequestSerialInterrupt()
{
	interruptRequests |= (1 << 3);
}

void CPU::RequestJoypadInterrupt()
{
	interruptRequests |= (1 << 4);
}

bool CPU::HandleInterrupts()
{
	// Todo: STAT interrupt, TIMA interrupt, Serial interrupt, Joypad interrupt
	// Todo: halt should be disabled as soon as an interrupt is pending (not necessarily handled)

	uint8_t interrupts = interruptEnable & interruptRequests;
	bool interrupt_handled = false;

	if (interrupts && haltMode)
	{
		haltMode = 0;
	}

	auto activate_interrupt = [&](uint8_t num) {
		_ASSERTE(num < 5 && !DMA_transfer);
		interruptRequests &= ~(1 << num);
		IME = 0;
		gameboy->UpdateClock(8);
		Push(Helpers::GetHighByte(programCounter));
		Push(Helpers::GetLowByte(programCounter));
		uint8_t interruptVector = 0x40 + num * 0x8;
		SetProgramCounter(interruptVector);
		interrupt_handled = true;
	};



	for (uint8_t i = 0; i < 5; i++)
	{
		if (Helpers::ExtractBit(interrupts, i) && IME)
		{
			activate_interrupt(i);
			break;
		}
	}

	return interrupt_handled;
}

void CPU::StartDMATransfer(uint16_t address)
{
	if (DMA_transfer) 
	{
		return;
	}

	DMA_transfer = true;
	DMA_start = cycles;
	DMA_address = address;
}

Instruction CPU::DecodeOpcode(uint8_t opcode)
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

void CPU::IncrementProgramCounter(int8_t offset)
{
	programCounter += offset;
	gameboy->UpdateClock(4);
}

void CPU::SetProgramCounter(uint16_t value)
{
	programCounter = value;
	gameboy->UpdateClock(4);
}

void CPU::Push(uint8_t value)
{
	stackPointer--;
	memory->Write(value, stackPointer);
}

void CPU::Push16(uint8_t opcode)
{
	uint16_t value;
	std::string reg_string;
	gameboy->UpdateClock(4);

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

	Push(Helpers::GetHighByte(value));
	Push(Helpers::GetLowByte(value));

	if (printLogs)
	{
		std::cout << "PUSH " << reg_string << "\n";
	}
}

uint8_t CPU::Pop()
{
	uint8_t result = memory->Read(stackPointer);
	stackPointer++;
	return result;
}

void CPU::Pop16(uint8_t opcode)
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

uint8_t CPU::GetRegisterValue(uint8_t num)
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
		return memory->Read(Get16BitRegisterValue(HL16));
	case A:
		return registerA;
	}
	_ASSERTE(false);
	return 0;
}

uint16_t CPU::Get16BitRegisterValue(uint8_t num)
{
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

void CPU::SetRegister(uint8_t num, uint16_t value)
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
		memory->Write(uint8_t(value), Get16BitRegisterValue(HL16));
		return;
	case A:
		registerA = uint8_t(value);
		return;
	}
	_ASSERTE(false);
}

void CPU::Set16BitRegister(uint8_t num, uint16_t value)
{
	switch (num)
	{
	case BC16:
		registerB = Helpers::GetHighByte(value);
		registerC = Helpers::GetLowByte(value);
		return;
	case DE16:
		registerD = Helpers::GetHighByte(value);
		registerE = Helpers::GetLowByte(value);
		return;
	case HL16:
		registerH = Helpers::GetHighByte(value);
		registerL = Helpers::GetLowByte(value);
		return;
	case SP16:
		stackPointer = value;
		return;
	}

	_ASSERTE(false);
}
uint8_t CPU::CheckCondition(uint8_t condition)
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

void CPU::ExecuteInstruction(uint8_t opcode)
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


Instruction CPU::DecodeCBOpcode(uint8_t opcode)
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
void CPU::ExecuteCBInstruction()
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

void CPU::LD(uint8_t opcode)
{
	uint8_t reg1 = (opcode & 0b00111000) >> 3;
	uint8_t reg2 = opcode & 0b00000111;
	SetRegister(reg1, GetRegisterValue(reg2));

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), reg1, reg2);
	}
}

void CPU::LDH(uint8_t opcode)
{
	uint8_t offset = ReadROMLine();

	if (opcode == 0xE0)
	{
		memory->Write(GetRegisterValue(A), 0xFF00 + offset);
		if (printLogs)
		{
			std::cout << InstructionToString(DecodeOpcode(opcode))
				<< std::format(" ${:X}, ", 0xFF00 + offset) << RegisterToString(A) << "\n";
		}
	}

	if (opcode == 0xF0)
	{
		SetRegister(A, memory->Read(0xFF00 + offset));
		if (printLogs)
		{
			PrintImmediateInstruction(DecodeOpcode(opcode), 0xFF00 + offset, A);
		}
	}
}

void CPU::LDtoIndirect(uint8_t opcode)
{
	std::string reg_string;
	switch (opcode)
	{
	case 0x02:
		memory->Write(registerA, Get16BitRegisterValue(BC16));
		reg_string = "(BC)";
		break;
	case 0x12:
		memory->Write(registerA, Get16BitRegisterValue(DE16));
		reg_string = "(DE)";
		break;
	case 0x22:
		memory->Write(registerA, Get16BitRegisterValue(HL16));
		Set16BitRegister(HL16, Get16BitRegisterValue(HL16) + 1);
		reg_string = "(HL+)";
		break;
	case 0x32:
		memory->Write(registerA, Get16BitRegisterValue(HL16));
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

void CPU::ImmediateLD16(uint8_t opcode)
{
	uint8_t reg = (opcode & 0b00110000) >> 4;
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t immediateValue = (uint16_t(msb) << 8) + lsb;
	Set16BitRegister(reg, immediateValue);

	if (printLogs)
	{
		PrintImmediateInstruction16(DecodeOpcode(opcode), immediateValue, reg);
	}
}


void CPU::IndirectLD(uint8_t opcode)
{
	std::string reg_string;
	switch (opcode)
	{
	case 0x0A:
		registerA = memory->Read(Get16BitRegisterValue(BC16));
		reg_string = "(BC)";
		break;
	case 0x1A:
		registerA = memory->Read(Get16BitRegisterValue(DE16));
		reg_string = "(DE)";
		break;
	case 0x2A:
		registerA = memory->Read(Get16BitRegisterValue(HL16));
		Set16BitRegister(HL16, Get16BitRegisterValue(HL16) + 1);
		reg_string = "(HL+)";
		break;
	case 0x3A:
		registerA = memory->Read(Get16BitRegisterValue(HL16));
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

void CPU::LDfromAccumulator(uint8_t opcode)
{
	uint16_t address = 0xFF00 + GetRegisterValue(C);
	memory->Write(GetRegisterValue(A), address);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), A, C);
	}
}

void CPU::LDAccumulator(uint8_t opcode)
{
	uint16_t address = 0xFF00 + GetRegisterValue(C);
	SetRegister(A, memory->Read(address));

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode), C, A);
	}
}

void CPU::LDtoAddress(uint8_t opcode)
{
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	memory->Write(registerA, address);

	if (printLogs)
	{
		PrintImmediateInstructionReversed(DecodeOpcode(opcode), address, A);
	}
}

void CPU::LDStackPointertoAddress(uint8_t opcode)
{
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	memory->Write(Helpers::GetLowByte(stackPointer), address);
	memory->Write(Helpers::GetHighByte(stackPointer), address + 1);

	if (printLogs)
	{
		std::cout << "LD " << std::format("${:X}, ", address) << "SP" << "\n";
	}
}

// todo: clean up/separate writes to memory
void CPU::LDfromAddress(uint8_t opcode)
{
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	SetRegister(A, memory->Read(address));

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), address, A);
	}
}

void CPU::LDStackPointerFromHL(uint8_t opcode)
{
	stackPointer = Get16BitRegisterValue(HL16);
	gameboy->UpdateClock(4);

	if (printLogs)
	{
		std::cout << "LD SP, HL" << "\n";
	}
}

void CPU::LDHLFromStackPointer(uint8_t opcode)
{
	int8_t offset = ReadROMLine();
	uint32_t result = stackPointer + offset;
	Set16BitRegister(HL16, stackPointer + offset);
	gameboy->UpdateClock(4);

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

void CPU::Add(uint8_t opcode)
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

void CPU::Add16(uint8_t opcode)
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
	gameboy->UpdateClock(4);

	if (printLogs)
	{
		PrintInstruction16(DecodeOpcode(opcode), HL16, reg);
	}
}

void CPU::ImmediateAdd(uint8_t opcode)
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

void CPU::ImmediateAddToStackPointer(uint8_t opcode)
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
	gameboy->UpdateClock(8);

	if (printLogs)
	{
		std::cout << "ADD SP, " << std::format("${:X}", offset) << "\n";
	}
}

void CPU::Adc(uint8_t opcode)
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

void CPU::ImmediateAdc(uint8_t opcode)
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

void CPU::SetFlag(uint8_t num, bool value)
{
	registerF &= ~(1 << num);
	registerF |= (uint8_t(value) << num);
}

bool CPU::GetFlag(uint8_t num)
{
	return registerF & (1 << num);
}

void CPU::Sub(uint8_t opcode)
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

void CPU::ImmediateSub(uint8_t opcode)
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

void CPU::SBC(uint8_t opcode)
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

void CPU::Compare(uint8_t opcode)
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

void CPU::ImmediateCompare(uint8_t opcode)
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

void CPU::RotateLeftAccumulator(uint8_t opcode)
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

void CPU::RotateRightAccumulator(uint8_t opcode)
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

void CPU::RotateLeft(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;

	uint8_t oldValue = GetRegisterValue(reg);
	bool oldCarry = GetFlag(CarryFlag);
	uint8_t newValue = (oldValue << 1) + uint8_t(oldCarry);

	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 7));
	SetRegister(reg, newValue);
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::RotateRight(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);

	bool oldCarry = GetFlag(CarryFlag);
	uint8_t newValue = (oldValue >> 1) + (uint8_t(oldCarry) << 7);

	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 0));
	SetRegister(reg, newValue);
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::RotateLeftThroughCarry(uint8_t reg)
{
	uint8_t oldValue = GetRegisterValue(reg);
	bool oldCarry = GetFlag(CarryFlag);
	uint8_t newValue = (oldValue << 1) + uint8_t(oldCarry);
	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 7));
	SetRegister(reg, newValue);
}

void CPU::RotateRightThroughCarry(uint8_t reg)
{
	uint8_t oldValue = GetRegisterValue(reg);
	bool oldCarry = GetFlag(CarryFlag);
	uint8_t newValue = (oldValue >> 1) + (uint8_t(oldCarry) << 7);
	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 0));
	SetRegister(reg, newValue);
}

void CPU::RotateLeftCircular(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = std::rotl(oldValue, 1);

	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 7));
	SetRegister(reg, newValue);
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::RotateRightCircular(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = std::rotr(oldValue, 1);

	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 0));
	SetRegister(reg, newValue);
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::RLCA(uint8_t opcode)
{
	uint8_t oldValue = GetRegisterValue(A);

	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 7));
	SetRegister(A, std::rotl(oldValue, 1));
	SetFlag(ZeroFlag, false); // TODO: figure out actual behavior
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), A);
	}
}

void CPU::RRCA(uint8_t opcode)
{
	uint8_t oldValue = GetRegisterValue(A);

	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 0));
	SetRegister(A, std::rotr(oldValue, 1));
	SetFlag(ZeroFlag, false); // TODO: fix
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), A);
	}
}

void CPU::DAA(uint8_t opcode)
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

	if (printLogs)
	{
		std::cout << "DAA\n";
	}
}

void CPU::SBCImmediate(uint8_t opcode)
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

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, A);
	}
}

void CPU::ShiftRightThroughCarry(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = oldValue >> 1;

	SetRegister(reg, newValue);
	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 0));
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::Increment(uint8_t opcode)
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

void CPU::Increment16(uint8_t opcode)
{
	uint8_t reg = (opcode & 0b00110000) >> 4;
	uint16_t value = Get16BitRegisterValue(reg);
	Set16BitRegister(reg, value + 1);
	gameboy->UpdateClock(4);

	if (printLogs)
	{
		PrintInstruction16(DecodeOpcode(opcode), reg);
	}
}

void CPU::Decrement(uint8_t opcode)
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

void CPU::Decrement16(uint8_t opcode)
{
	uint8_t reg = (opcode & 0b00110000) >> 4;
	uint16_t value = Get16BitRegisterValue(reg);
	std::string reg_string;
	Set16BitRegister(reg, value - 1);
	gameboy->UpdateClock(4);

	switch (reg)
	{
	case 0:
		reg_string = "BC";
		break;
	case 1:
		reg_string = "DE";
		break;
	case 2:
		reg_string = "HL";
		break;
	case 3:
		reg_string = "SP";
		break;
	}

	if (printLogs)
	{
		std::cout << "DEC16 " << reg_string << "\n";
	}
}

void CPU::And(uint8_t opcode)
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

void CPU::ImmediateAnd(uint8_t opcode)
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
void CPU::Or(uint8_t opcode)
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

void CPU::ImmediateOr(uint8_t opcode)
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

void CPU::Xor(uint8_t opcode)
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

void CPU::ImmediateXor(uint8_t opcode)
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

void CPU::ComplementCarryFlag(uint8_t opcode)
{
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);
	SetFlag(CarryFlag, !GetFlag(CarryFlag));
}

void CPU::SetCarryFlag(uint8_t opcode)
{
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);
	SetFlag(CarryFlag, 1);
}

void CPU::ComplementAccumulator(uint8_t opcode)
{
	SetRegister(A, ~GetRegisterValue(A));
	SetFlag(NegativeFlag, 1);
	SetFlag(HalfCarryFlag, 1);
}

void CPU::ImmediateLD(uint8_t opcode)
{
	uint8_t reg1 = (opcode & 0b00111000) >> 3;
	uint8_t immediateValue = ReadROMLine();
	SetRegister(reg1, immediateValue);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), immediateValue, reg1);
	}
}

void CPU::Jump(uint8_t opcode)
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

void CPU::JumpToHL(uint8_t opcode)
{
	// No PC write delay
	programCounter = Get16BitRegisterValue(HL16);

	if (printLogs)
	{
		std::cout << InstructionToString(DecodeOpcode(opcode)) << "\n";
	}
}

void CPU::ConditionalJump(uint8_t opcode)
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

void CPU::RelativeJump(uint8_t opcode)
{
	int8_t offset = ReadROMLine();
	IncrementProgramCounter(offset);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), programCounter);
	}
}

void CPU::ConditionalRelativeJump(uint8_t opcode)
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
void CPU::FunctionCall(uint8_t opcode)
{
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	uint8_t current_msb = Helpers::GetHighByte(programCounter);
	uint8_t current_lsb = Helpers::GetLowByte(programCounter);
	Push(current_msb);
	Push(current_lsb);
	SetProgramCounter(address);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), address);
	}
}

void CPU::ConditionalFunctionCall(uint8_t opcode)
{
	uint8_t condition = (opcode & 0b00011000) >> 3;
	uint8_t lsb = ReadROMLine();
	uint8_t msb = ReadROMLine();
	uint16_t address = (uint16_t(msb) << 8) + lsb;
	uint8_t current_msb = Helpers::GetHighByte(programCounter);
	uint8_t current_lsb = Helpers::GetLowByte(programCounter);

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

void CPU::FunctionReturn(uint8_t opcode)
{
	uint8_t lower_byte = Pop();
	uint8_t upper_byte = Pop();
	SetProgramCounter(lower_byte + (upper_byte << 8));

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeOpcode(opcode), programCounter);
	}
}

void CPU::ConditionalFunctionReturn(uint8_t opcode)
{
	uint8_t condition = (opcode & 0b00011000) >> 3;
	gameboy->UpdateClock(4);
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

void CPU::CPL(uint8_t opcode)
{
	SetRegister(A, ~GetRegisterValue(A));
	SetFlag(NegativeFlag, true);
	SetFlag(HalfCarryFlag, true);
	if (printLogs)
	{
		std::cout << "CPL\n";
	}
}

void CPU::SCF(uint8_t opcode)
{
	SetFlag(CarryFlag, true);
	SetFlag(NegativeFlag, false);
	SetFlag(HalfCarryFlag, false);
}

void CPU::CCF(uint8_t opcode)
{
	SetFlag(CarryFlag, !GetFlag(CarryFlag));
	SetFlag(NegativeFlag, false);
	SetFlag(HalfCarryFlag, false);
}



void CPU::InterruptReturn(uint8_t opcode)
{
	IME = 1;
	FunctionReturn(opcode);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void CPU::Restart(uint8_t opcode)
{
	uint16_t address = ((opcode & 0b00111000) >> 3) * 8;
	uint8_t current_msb = Helpers::GetHighByte(programCounter);
	uint8_t current_lsb = Helpers::GetLowByte(programCounter);
	Push(current_msb);
	Push(current_lsb);
	SetProgramCounter(address);

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void CPU::Halt(uint8_t opcode)
{
	haltMode = true;
	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void CPU::Stop(uint8_t opcode)
{
	systemClockActive = 0;
	mainClockActive = 0;
	memory->Set(0xFF04, 0);

	if (printLogs)
	{
		std::cout << "Stop\n";
	}
}

void CPU::DisableInterrupts(uint8_t opcode)
{
	IME = 0;

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void CPU::EnableInterrupts(uint8_t opcode)
{
	IME = 1;

	if (printLogs)
	{
		PrintInstruction(DecodeOpcode(opcode));
	}
}

void CPU::Bit(uint8_t opcode)
{
	uint8_t bit = (opcode - 0x40) / 8;
	uint8_t reg = (opcode % 0x08);
	bool result = Helpers::ExtractBit(GetRegisterValue(reg), bit);

	SetFlag(ZeroFlag, !result);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 1);

	if (printLogs)
	{
		PrintImmediateInstruction(DecodeCBOpcode(opcode), bit, reg);
	}
}

void CPU::SLA(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = oldValue << 1;
	SetRegister(reg, newValue);

	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 7));
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::SRA(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = (oldValue >> 1) + (oldValue & (1u << 7)); // MSB needs to stay the same

	SetRegister(reg, newValue);
	SetFlag(CarryFlag, Helpers::ExtractBit(oldValue, 0));
	SetFlag(ZeroFlag, newValue == 0);
	SetFlag(NegativeFlag, 0);
	SetFlag(HalfCarryFlag, 0);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::SET(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t bit = (opcode - 0xC0) / 8;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = oldValue | (1 << bit);
	SetRegister(reg, newValue);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::RES(uint8_t opcode)
{
	uint8_t reg = opcode & 0b00000111;
	uint8_t bit = (opcode - 0x80) / 8;
	uint8_t oldValue = GetRegisterValue(reg);
	uint8_t newValue = oldValue & (~(1 << bit));
	SetRegister(reg, newValue);

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::SWAP(uint8_t opcode)
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

	if (printLogs)
	{
		PrintInstruction(DecodeCBOpcode(opcode), reg);
	}
}

void CPU::Nop(uint8_t opcode)
{
	if (printLogs)
	{
		std::cout << "Nop\n";
	}
}

void CPU::UpdateClock(uint32_t cycleCount)
{
	cycles += cycleCount;

	if (DMA_transfer && cycles - DMA_start > 640)
	{
		DMA_transfer = false;
		for (uint16_t offset = 0; offset < 160; offset++)
		{
			uint8_t value = memory->Get(DMA_address + offset);
			memory->Set(0xFE00 + offset, value);
		}
	}

	uint8_t previous_div = dividerRegister;
	uint16_t previous_div_clock = divClock;
	divClock += cycleCount;
	dividerRegister = Helpers::GetHighByte(divClock);
	if (dividerRegister != previous_div)
	{
		// DIV-APU clock increments on falling edges of bit 4
		if (Helpers::ExtractBit(previous_div, 4) && !Helpers::ExtractBit(dividerRegister, 4))
		{
			apu->StepAPU();
		}
	}


	auto increment_timer = [&]() {
		if (timerCounter == 0xFF)
		{
			RequestTimerInterrupt();
			timerCounter = timerModulo;
		}
		else
		{
			timerCounter++;
		}
	};

	switch (timerControl & 0b11)
	{
	case 0:
	{
		// clock / 1024
		if ((Helpers::ExtractBit(previous_div_clock, 9) && previousTimerControl) && !(Helpers::ExtractBit(divClock, 9) && Helpers::ExtractBit(timerControl, 2)))
		{
			increment_timer();
		}
		break;
	}

	case 1:
	{
		// clock / 16
		if ((Helpers::ExtractBit(previous_div_clock, 3) && previousTimerControl) && !(Helpers::ExtractBit(divClock, 3) && Helpers::ExtractBit(timerControl, 2)))
		{
			increment_timer();
		}
		break;
	}
	case 2:
	{
		// clock / 64
		if ((Helpers::ExtractBit(previous_div_clock, 5) && previousTimerControl) && !(Helpers::ExtractBit(divClock, 5) && Helpers::ExtractBit(timerControl, 2)))
		{
			increment_timer();
		}
		break;
	}
	case 3:
	{
		// clock / 256
		if ((Helpers::ExtractBit(previous_div_clock, 7) && previousTimerControl) && !(Helpers::ExtractBit(divClock, 7) && Helpers::ExtractBit(timerControl, 2)))
		{
			increment_timer();
		}
		break;
	}
	}

	previousTimerControl = Helpers::ExtractBit(timerControl, 2);
}

uint8_t CPU::ReadROMLine()
{
	return memory->Read(programCounter++);
}

std::string CPU::RegisterToString(uint8_t reg)
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

std::string CPU::RegisterToString16(uint8_t reg)
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

std::string CPU::InstructionToString(Instruction instruction)
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

void CPU::PrintInstruction(Instruction instruction, uint8_t reg1, uint8_t reg2)
{
	std::string instruction_string = InstructionToString(instruction);
	std::string reg1_string = reg1 == 0xFF ? "" : RegisterToString(reg1);
	std::string reg2_string = reg2 == 0xFF ? "" : ", " + RegisterToString(reg2);

	std::cout << instruction_string << " " << reg1_string << reg2_string << "\n";
}

void CPU::PrintInstruction16(Instruction instruction, uint8_t reg1, uint8_t reg2)
{
	std::string instruction_string = InstructionToString(instruction);
	std::string reg1_string = reg1 == 0xFF ? "" : RegisterToString16(reg1);
	std::string reg2_string = reg2 == 0xFF ? "" : ", " + RegisterToString16(reg2);

	std::cout << instruction_string << " " << reg1_string << reg2_string << "\n";
}

void CPU::PrintImmediateInstruction(Instruction instruction, uint16_t immediateValue, uint8_t reg1)
{
	std::string instruction_string = InstructionToString(instruction);
	std::string reg1_string = reg1 == 0xFF ? "" : RegisterToString(reg1) + ", ";

	std::cout << instruction_string << " " << reg1_string << std::format("${:X}", immediateValue) << "\n";
}

void CPU::PrintImmediateInstruction16(Instruction instruction, uint16_t immediateValue, uint8_t reg1)
{
	std::string instruction_string = InstructionToString(instruction);
	std::string reg1_string = reg1 == 0xFF ? "" : RegisterToString16(reg1) + ", ";

	std::cout << instruction_string << " " << reg1_string << std::format("${:X}", immediateValue) << "\n";
}

void CPU::PrintImmediateInstructionReversed(Instruction instruction, uint16_t immediateValue, uint8_t reg1)
{
	std::string instruction_string = InstructionToString(instruction);
	std::string reg1_string = reg1 == 0xFF ? "" : RegisterToString(reg1);

	std::cout << instruction_string << " " << std::format("${:X}", immediateValue) << ", " << reg1_string << "\n";
}

std::string CPU::ConditionToString(uint8_t condition)
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

void CPU::LogCPUState()
{
	cpu_state << std::format("A:{:02X} F:{:02X} B:{:02X} C:{:02X} D:{:02X} E:{:02X} H:{:02X} L:{:02X} SP:{:04X} PC:{:04X} PCMEM:{:02X},{:02X},{:02X},{:02X}, CYCLE:{:d} TIMER:{:02X} TIMER_CONTROL:{:02X}",
		registerA, registerF, registerB, registerC, registerD, registerE, registerH, registerL, stackPointer, programCounter,
		memory->Get(programCounter), memory->Get(programCounter + 1), memory->Get(programCounter + 2), memory->Get(programCounter + 3), cycles, memory->Get(0xFF05), memory->Get(0xFF07)) << std::endl;

}

void CPU::LogCPUStateDetailed()
{
	cpu_state << std::format("A:{:02X} F:{:02X} B:{:02X} C:{:02X} D:{:02X} E:{:02X} H:{:02X} L:{:02X} SP:{:04X} SPMEM:{:02X},{:02X} PC:{:04X} PCMEM:{:02X},{:02X},{:02X},{:02X}, CYCLE:{:d}, PPU_CYCLE:{:d}, LY:{:02X} TIMER:{:02X} TIMER_CONTROL:{:02X} LCDC:{:d}",
		registerA, registerF, registerB, registerC, registerD, registerE, registerH, registerL, stackPointer, memory->Get(stackPointer), memory->Get(stackPointer + 1), programCounter,
		memory->Get(programCounter), memory->Get(programCounter + 1), memory->Get(programCounter + 2), memory->Get(programCounter + 3), cycles, ppu->clock, memory->Get(0xFF44), 
		memory->Get(0xFF05), memory->Get(0xFF07), Helpers::ExtractBit(memory->Get(0xFF40), 7)) << std::endl;
}