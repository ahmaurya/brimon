/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "Sense.h"

#ifndef COMMAND_H
#define COMMAND_H

/**
 * various command codes as used by the commandline application
 * and the desktop Java application
 */

enum {
	BEACON = 0,
	ROUTING = 1,
	TIMESYNC_START = 2,
	TIMESYNC_STOP = 3,
	SENSE = 4,
	COLLECT = 5,
	TRANSFER = 6,
	ROUTING_COMPLETE = 7,
	TIMESYNC_START_COMPLETE = 8,
	TIMESYNC_STOP_COMPLETE = 9,
	SENSE_COMPLETE = 10,
	COLLECT_COMPLETE = 11,
	TRANSFER_COMPLETE = 12,
	DIFF_COMMAND_COMPLETE = 6,
};

#endif
