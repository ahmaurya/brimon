/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */


/*
	This file provides TimeSync Interface.
	It provides two command. start() is used to start the duty cycle. stop() is used to stop duty cycle.
	startDone() and stopDone() events are signaled at the end of start() and stop()

*/
interface TimeSync
{
	command void start();
	command void stop();
	event void startDone(error_t error);
	event void stopDone(error_t error);
}
