#include <byte_framing.h>
#include <iostream.h>

struct byteFrameData retVar;

void byteFraming_init(uint8_t _escapeByte, uint8_t _startByte, uint8_t _endByte) {
	extern byteFraming_escapeByte = _escapeByte;
	extern byteFraming_startByte = _startByte;
	extern byteFraming_endByte = _endByte;
}

byteFrameData byteFraming_checkByte(uint8_t _inputData) {
	retVar.data = _inputData;
	retVar.byteFraming_escapeByte = 0x00;
	switch (_inputData) {
		case byteFraming_escapeByte:
		case byteFraming_startByte:
		case byteFraming_endByte:
		//Place escape character at the back, so that when we transmit, it is easier to pop and send the escape char first
			retVar.data ^= byteFraming_escapeByte;
			retVar.byteFraming_escapeByte = byteFraming_escapeByte;
			break;
	}
	return retVar;
}

// uint8_t byteFraming_removeZeroPadding(uint16_t _inputData) {
// 	return uint8_t(_inputData >> 8);
// }

/*
byteFraming_init(0xD3, 0xCC, 0x55); //escape byte 0x5A, flag byte 0xCC
for eachbyte in dataframe {
	result16Byte = byteFraming_checkByte(eachByte);
	if uint8_t(result16Byte) == 0x00 {
		put_byte(byteFraming_removeZeroPadding(result16Byte))
	}
	else {
		put_byte(uint8_t(result16Byte));
		put_byte(uint8_t(result16Byte >> 8));
	}
}
*/