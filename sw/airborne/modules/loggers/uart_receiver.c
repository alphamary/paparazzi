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
#include "subsystems/navigation/waypoints.h"
#include "generated/flight_plan.h"

static const uint8_t START = 0x99;
static const uint8_t END = 0x55;
static const uint8_t ESC = 0xD3;
static const uint8_t velocityEvent = 0x11;
static const uint8_t positionEvent = 0x22;

struct uart_receiver_data_struct received_data;
void uart_receiver_init(void){
	
}
void uart_receiver_periodic(void){
//extern void waypoint_set_enu_i(uint8_t wp_id, struct EnuCoor_i *enu);

	struct EnuCoor_i goalCmd;
	uint8_t temp;
	uint8_t receivedByte;
	uint16_t bufferSize = uart_char_available(&uart2);
//	uart_put_byte(&uart2, bufferSize);
	if(bufferSize > 0){	
		for(int i = 0; i < bufferSize; i++){
			receivedByte = uart_getch(&uart2);			
			if(receivedByte == START){
				uart_put_byte(&uart2, 0, START);
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
					
				uint8_t *test = (uint8_t*) &received_data;
				for (int i=0; i<13; i++){
    					uart_put_byte(&uart2, 0, test[i]);
    				 }
			//	uart_put_byte(&uart2, 0, received_data.cmd_x);

				if (received_data.event ==velocityEvent){
				
				} 
				else if (received_data.event == positionEvent){
					goalCmd.x = received_data.cmd_x;
					goalCmd.y = received_data.cmd_y;
					goalCmd.z = received_data.cmd_z;
					waypoint_move_enu_i(WP_ROSINTERFACE, &goalCmd);
				}
				
			}
		
		}

	}
}

/*
send commands to paparazzi functions 
 EVENT | COMMAND_X | COMMAND_Y | COMMAND_Z  
*/

//extern void waypoint_set_enu_i(uint8_t wp_id, struct EnuCoor_i *enu);
//wp_id = ROSINTERFACE
