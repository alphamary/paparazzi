/*
 * Copyright (C) 2015 The Paparazzi Team
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

#ifndef LOGGER_UART_H_
#define LOGGER_UART_H_

#include "std.h"

extern void logger_uart_init(void);
extern void logger_uart_periodic(void);

#define PACKED __attribute__((__packed__))

//union {
  struct PACKED logger_uart_data_struct {
    uint8_t start;
    uint16_t id;         // 1
    float timeStamp;
    unsigned char deviceID;
    float acc_x;      // 5
    float acc_y;
    float acc_z;
    float vel_x;      // 5
    float vel_y;
    float vel_z;
    float pos_x;      // 5
    float pos_y;
    float pos_z;
    uint16_t crc;
    uint8_t end;
  };
//  uint8_t bytes[23];
//} logger_uart_data_union;

#endif /* LOGGER_UART_H_ */
