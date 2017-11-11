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

#include "logger_uart.h"

#include "state.h"
#include "led.h"
#include "subsystems/imu.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "modules/ins/imu_chimu.h"
#include "modules/loggers/byte_framing.h"


struct logger_uart_data_struct logger_uart_data;
CHIMU_PARSER_DATA CHIMU_DATA;

void logger_uart_init(void)
{
  logger_uart_data.id = 0;
}
void logger_uart_periodic(void)
{
  logger_uart_data.start = 0x99;
  logger_uart_data.end = 0x55;
  uint8_t esc = 0xD3;
  logger_uart_data.id++;
  struct NedCoor_f accel = *stateGetAccelNed_f();
  struct NedCoor_f vel = *stateGetSpeedNed_f();
  struct NedCoor_f pos = *stateGetPositionNed_f();
  if (logger_uart_data.id & 0x0080) 
  {
    LED_ON(1);
  } 
  else 
  {
    LED_OFF(1);
  }
  
  logger_uart_data.timeStamp = (float)(sys_time.nb_sec + (float)((sys_time.nb_sec_rem) *1000) / sys_time.cpu_ticks_per_sec);
  //logger_uart_data.timeStamp = (int)(get_sys_time_float() * 1000);
  logger_uart_data.deviceID = CHIMU_DATA.m_DeviceID;
  logger_uart_data.acc_x = accel.x;
  logger_uart_data.acc_y = accel.y;
  logger_uart_data.acc_z = accel.z;
  logger_uart_data.vel_x = vel.x;
  logger_uart_data.vel_y = vel.y;
  logger_uart_data.vel_z = vel.z;
  logger_uart_data.pos_x = pos.x;
  logger_uart_data.pos_y = pos.y;
  logger_uart_data.pos_z = pos.z;
  uint16_t crc = 0;
  uint8_t *p = (uint8_t*) &logger_uart_data; 
  uart_put_byte(&uart2, logger_uart_data.start); 
  for (int i=1; i<43; i++){
  if(p[i]==logger_uart_data.start || p[i]==logger_uart_data.end || p[i]==esc){
    uart_put_byte(&uart2, esc);
    crc += esc;
    uint8_t escData = (p[i])^esc;
    crc += escData;
    uart_put_byte(&uart2,escData);
  }
  else{
    uart_put_byte(&uart2,p[i]);
    crc += p[i];

  }
 }
  
  uart_put_byte(&uart2, crc&0xFF);
  uart_put_byte(&uart2, (crc>>8)&0xFF);

  uart_put_byte(&uart2, logger_uart_data.end); 

}
