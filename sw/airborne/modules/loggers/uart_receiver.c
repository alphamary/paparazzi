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
 * Boston, MA 0211-307, USA.
 *
 */
/*
________________________packet________________________________

uint8_t start byte, uint8_t event, float commandX, float commandY, float commandZ, uint6_t crc, uint8_t end byte
______________________________________________________________

	commandX can be velocityX or positionX
	commandY can be velocityY or positionY
	commandZ can be velocityZ or positionZ
________________________event byte____________________________

0x1 for set velocity
0x22 for set position
______________________________________________________________

*/
#include "uart_receiver.h"
#include "state.h"
#include "mcu_periph/uart.h"
#include "string.h"
#include "subsystems/navigation/waypoints.h"
#include "generated/flight_plan.h"

static const uint8_t START = 0x99;
static const uint8_t END = 0x55;
static const uint8_t ESC = 0xD3;
static const uint8_t velocityEvent = 17;
static const uint8_t positionEvent = 34;

struct uart_receiver_data_struct received_data;
void uart_receiver_init(void){
	
}
void uart_receiver_periodic(void){
//extern void waypoint_set_enu_i(uint8_t wp_id, struct EnuCoor_i *enu);

	struct EnuCoor_i goalCmd;
	uint8_t temp;
	uint8_t receivedByte;
	uint16_t bufferSize = uart_char_available(&uart2);
	uint8_t *p = (uint8_t*) &received_data; 

//	uart_put_byte(&uart2, bufferSize);
//	uart_put_byte(&uart2, 0, bufferSize);
	temp = 0;
	if(bufferSize > 12){
		for(int i = 0; i < 13; i++){
			receivedByte = uart_getch(&uart2);			
			if(receivedByte != END){
				if (receivedByte == START){
					temp = 0;
				}	
				else if (receivedByte == ESC){
					receivedByte = uart_getch(&uart2);
					i++;
					p[temp] = receivedByte ^ ESC;
					temp++;
				}
				else{							
					p[temp] = receivedByte;
					temp++;
				}
			}
		}
	}
		
		uint16_t crccheck = 0; 	
	//	uint8_t *check = (uint8_t*) &received_data; 
		for (int i=0; i<13; i++){
			//uart_put_byte(&uart2, 0, p[i]);
    			crccheck += p[i] ;
    		}
		//if (received_data.crc == crccheck){
			//uart_put_byte(&uart2, 0, (received_data.event));
			//uart_put_byte(&uart2, 0, (crccheck - received_data.crc));
			//uart_put_byte(&uart2, 0, positionEvent - received_data.event);
			if (received_data.event == velocityEvent){
		
			} 
			else if (received_data.event == positionEvent){
				//uart_put_byte(&uart2, 0, positionEvent);
				goalCmd.x = received_data.cmd_x;
				goalCmd.y = received_data.cmd_y;
				goalCmd.z = received_data.cmd_z;
				waypoint_move_enu_i(WP_ROSINTERFACE, &goalCmd);
			}
			memset(&received_data, 0, sizeof(received_data));
		//}
}

/*
send commands to paparazzi functions 
 EVENT | COMMAND_X | COMMAND_Y | COMMAND_Z | crc 
*/

//extern void waypoint_set_enu_i(uint8_t wp_id, struct EnuCoor_i *enu);
//wp_id = ROSINTERFACE
