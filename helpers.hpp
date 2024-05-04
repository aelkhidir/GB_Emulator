#include<format>

namespace Helpers
{
	inline bool ExtractBit(uint16_t value, uint8_t bit)
	{
		return (value >> bit) & 0x01;
	}

	inline uint8_t GetHighByte(uint16_t value)
	{
		return (value & 0xff00) >> 8;
	}

	inline uint8_t GetLowByte(uint16_t value)
	{
		return value & 0x00ff;
	}
}