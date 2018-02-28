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

*/
#include "uart_receiver.h"
#include "state.h"
#include "mcu_periph/uart.h"
#include "string.h"

static const uint8_t START = 0x99;
static const uint8_t END = 0x55;
static const uint8_t ESC = 0xD3;
static const char ACTION[2] = {char(0x11), char(0x22)};

struct uart_receiver_data_struct received_data;
void uart_receiver_init(void){
	
}
void uart_receiver_periodic(void){
	struct NedCoor_f posGoal = *stateSetPositionNed_f();
	struct NedCoor_f velGoal = *stateSetSpeedNed_f();

	uint8_t temp;
	uint8_t receivedByte;
	uint16_t bufferSize = uart_char_available(&uart2);
	uart_put_byte(&uart2, bufferSize);
	if(bufferSize > 0){	
		for(int i = 0; i < bufferSize; i++){
			receivedByte = uart_getch(&uart2);			
			if(receivedByte == START){
				temp = 0;
				receivedByte = uart_getch(&uart2);
				i++;
				uint8_t *p = (uint8_t*) &received_data; 
				while(receivedByte != END){
					if(receivedByte == ESC){
						receivedByte = uart_getch(&uart2);
						i++;
						p[temp] = receivedByte ^ ESC;
						temp++;
						receivedByte = uart_getch(&uart2);
						i++;
					}
					else{
						p[temp] = receivedByte;
						temp++;
						receivedByte = uart_getch(&uart2);
						i++;
					}	
				}

				if (received_data.event == ACTION[0]){
					velGoal.x = received_data.cmd_x;
					velGoal.y = received_data.cmd_y;
					velGoal.z = received_data.cmd_z;
				} 
				else if (received_data.event == ACTION[1]){
					posGoal.x = received_data.cmd_x;
					posGoal.y = received_data.cmd_y;
					posGoal.z = received_data.cmd_z;
				}
				
	}
		
	}

	}

/*
send commands to paparazzi functions 
 EVENT | COMMAND_X | COMMAND_Y | COMMAND_Z  
*/

// static inline void stateSetAccelNed_f(struct NedCoor_f *ned_accel)
// /// Set ground speed in local NED coordinates (float).
// static inline void stateSetSpeedNed_f(struct NedCoor_f *ned_speed)
// /// Set position from local NED coordinates (float).
// static inline void stateSetPositionNed_f(struct NedCoor_f *ned_pos)
// */
