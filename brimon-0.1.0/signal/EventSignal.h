/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#ifndef EVENTSIGNAL_H
#define EVENTSIGNAL_H

typedef nx_struct radio_beacon_msg {
  nx_uint16_t type;
  nx_uint16_t rssi;
} radio_beacon_msg_t;

enum {
  AM_RADIO_BEACON_MSG = 6,
  BEACON_MSG = 0,
  BEACON_CHANNEL = 15
};

#endif
