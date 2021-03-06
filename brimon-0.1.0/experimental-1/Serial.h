/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

/** the common packet format used by the nesC program running on the mote
 * and the Java program running on the system and communicating with the
 * mote on a serial port. The Java program uses the Java data class SerialMsg
 * generated by the nesC toll mig. See makefile for more details of the usage
 * of mig command. See Command.h for the various values of the type field.
 */

#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

typedef nx_struct test_serial_msg {
	nx_uint16_t type;
	nx_uint32_t counter;
	nx_uint8_t payload[30];
} test_serial_msg_t;

enum {
  AM_TEST_SERIAL_MSG = 100,
};

#endif
