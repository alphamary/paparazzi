/*
 * Copyright (C) 2005-2013 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */
/*
________________________packet________________________________

uint8_t start byte, uint8_t event, float commandX, float commandY, float commandZ, uint8 or 16_t crc, uint8_t end byte
______________________________________________________________

	commandX can be velocityX or positionX
	commandY can be velocityY or positionY
	commandZ can be velocityZ or positionZ
________________________event byte____________________________

0x11 for set velocity
0x22 for set position
______________________________________________________________

TASK:

1) To receive data byte by byte
2) If received byte is 'start byte', start unframing.
									 untill it detects 'end byte'.
									 check received data sum with crc byte
3) Expected final result : event, commandX, commandY, command Z in struct received_data

note: I am currently not sending crc byte. It will be probably one byte long
______________________________________________________________

*/
#include "uart_receiver.h"
#include "state.h"
#include "mcu_periph/uart.h"
#include "string.h"

/*
enum DATACASE{
	STARTBYTE = char(0x99), 
	DATABYTE = char(0x55), 
	ENDBYTE = char(0xD3)
};
*/
static const uint8_t START = 0x99;
static const uint8_t END = 0x55;
static const uint8_t ESC = 0xD3;
static const char ACTION[2] = {char(0x11), char(0x22)};

struct uart_receiver_data_struct received_data;
void uart_receiver_init(void){
	
}
void uart_receiver_periodic(void){
	uint8_t receivedByte;
	uint8_t temp = 0;
	uint16_t bufferSize = uart_char_available(&uart2);
	char dataInput[bufferSize];
	uart_put_byte(&uart2, bufferSize);
	if(bufferSize > 0){	
		for(int i = 0; i < bufferSize; i++){
			receivedByte = uart_getch(&uart2);			
			if(receivedByte == START){
				receivedByte = uart_getch(&uart2);
				i++;
			while(receivedByte != END){
				if(receivedByte == ESC){
					receivedByte = uart_getch(&uart2);
					i++;
					dataInput[temp] = receivedByte ^ ESC;
					temp++;
					receivedByte = uart_getch(&uart2);
					i++;
				}
				else{
					dataInput[temp] = receivedByte;
					temp++;
					receivedByte = uart_getch(&uart2);
					i++;
				}		
			}
		}
		
	}

	}

/*
send commands to paparazzi functions 
 EVENT | COMMAND_X | COMMAND_Y | COMMAND_Z  
*/

	for(int j = 0; j<temp ; j++){
//		uart_put_byte(&uart2, dataInput[j]);
		if (dataInput[j] == ACTION[0]){
			
		}
		if (dataInput[j] == ACTION[1]){
		}
	}
	

}

/*
 struct PACKED uart_receiver_data_struct {
    uint8_t event;
    float cmd_x;      // 5
    float cmd_y;
    float cmd_z;


		switch(currentCase) {
			case STARTBYTE:
				if(receivedByte == START) {
					currentCase = DATABYTE;
				}
			break;
			case DATABYTE:
				switch(receivedByte) {
					case ESC:
						dataInput += uart_getch(&uart2) ^ ESC; //XOR
					break;
					case START:
						dataInput = "";
					break;
					case END:
						currentCase = ENDBYTE;
					break;
					default:
						dataInput += receivedByte; 
					break;
				}
			break;
			case ENDBYTE:
				for (int i = 0; i < strlen(dataInput); i++)
				{
					uart_put_byte(&uart2, dataInput[i]);
				}
			break;
			default:
				continue;
			break;
	
		}
	}
}



*/		
// quitFlag = True
		// rawData = ''	
		// while quitFlag:
		// 	newByte = (ser.read(1))		
		// 	if newByte == END_BYTE:
		// 		quitFlag = False
		// 	elif newByte == ESC_BYTE:
		// 		rawData += chr((ord(ser.read(1))^ord(ESC_BYTE)))
		// 	elif newByte == START_BYTE:
		// 		rawData = ''			
		// 	else:				
		// 		rawData += (newByte)
		// return rawData

/*
static inline void stateSetAccelNed_f(struct NedCoor_f *ned_accel)
/// Set ground speed in local NED coordinates (float).
static inline void stateSetSpeedNed_f(struct NedCoor_f *ned_speed)
/// Set position from local NED coordinates (float).
static inline void stateSetPositionNed_f(struct NedCoor_f *ned_pos)
*/
