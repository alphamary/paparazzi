/*!
 * \author FINken ROSInterface Project
 */

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

#ifndef UART_RECEIVER_H_
#define UART_RECEIVER_H_

#include "std.h"

extern void uart_receiver_init(void);
extern void uart_receiver_periodic(void);

#define PACKED __attribute__((__packed__))
/**
 * UART2 receiver structure 
 */
  struct PACKED uart_receiver_data_struct {
    uint8_t event; /*!< Event might be position command or velocity command, 0x22 and 0x11 respectively*/
    float cmd_x; /*!< cmd_x stands for position or velocity in x-direction of the coordinate*/      
    float cmd_y; /*!< cmd_y stands for position or velocity in y-direction of the coordinate*/
    float cmd_z; /*!< cmd_z stands for position or velocity in z-direction of the coordinate*/
    uint16_t crc;
  };

#endif /* UART_RECEIVER_H_ */
