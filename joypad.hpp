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

class Memory;
class CPU;

class Joypad
{
public:
	Joypad(Memory* memory, CPU* cpu) : memory(memory), cpu(cpu) {}
	Memory* memory;
	CPU* cpu;

	uint8_t joypad;
	uint8_t previous_joypad;


	void UpdateJoypadInput();
};