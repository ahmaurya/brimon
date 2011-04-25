/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

typedef nx_struct test_serial_msg {
  nx_uint16_t counter;
} test_serial_msg_t;

typedef nx_struct test_serial_msg_pctomote {
  nx_uint8_t command;
} test_serial_msg_pctomote_t;

typedef nx_struct test_serial_msg_motetopc {
  nx_uint8_t data;
} test_serial_msg_motetopc_t;

enum {
  AM_TEST_SERIAL_MSG = 0x89,
};

#endif
