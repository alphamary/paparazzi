/*!
 * \author FINken ROSInterface Project
 */

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
/** UART2 sender structure */
  struct PACKED logger_uart_data_struct {
    uint8_t start; /*!< Used in byte framing, it is defined as 0x99 in FINken ROSInterface project*/
    uint16_t id;  /*! UART2 package ID*/
    float timeStamp; /*! Current time in seconds*/
    unsigned char deviceID; /*!< ID of copter */
    float acc_x;    /*! Acceleration in x-coordinate*/
    float acc_y;    /*! Acceleration in y-coordinate*/
    float acc_z;    /*! Acceleration in y-coordinate*/
    float vel_x;    /*! Velocity in x-coordinate*/
    float vel_y;    /*! Velocity in y-coordinate*/
    float vel_z;    /*! Velocity in z-coordinate*/
    float pos_x;    /*! Position in x-coordinate*/
    float pos_y;    /*! Position in y-coordinate*/
    float pos_z;    /*! Position in z-coordinate*/
    uint16_t crc;   /*! Checksum (used in byte framing)*/
    uint8_t end; /*!< Used in byte framing, it is defined as 0x55 in FINken ROSInterface project*/
  };

#endif /* UART_SENDER_H_ */