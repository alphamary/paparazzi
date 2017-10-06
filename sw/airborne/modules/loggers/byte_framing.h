
#ifndef BYTE_FRAMING_H_
#define BYTE_FRAMING_H_

#include "std.h"

struct byteFrameData {
	uint8_t data;
	uint8_t byteFraming_escapeByte;
};

extern void byteFraming_init(uint8_t _escapeByte, uint8_t _startByte, uint8_t _endByte);
extern uint16_t byteFraming_checkByte(uint8_t _inputData);
extern uint8_t byteFraming_removeZeroPadding(uint16_t _inputData);

#endif /* BYTE_FRAMING_H_ */