/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "TimeSync.h"

module TimeSyncAppP {
	uses {
		interface TimeSync;
		interface Leds;
		interface Boot;
		interface Timer<TMilli> as Timer;
		interface SplitControl as AMControl;
	}
}

implementation {
	uint8_t flag=0;

	event void Boot.booted()
	{
		call Timer.startPeriodic(30000);
		call AMControl.start();
	}

	event void AMControl.startDone(error_t err)
	{
		if (err != SUCCESS)
		{
			call AMControl.start();
    	}
  	}

  	event void AMControl.stopDone(error_t err) {}

	event void Timer.fired()
	{
		if(flag == 0)
			call TimeSync.start();
		else
			call TimeSync.stop();
		flag=!flag;
	}

	event void TimeSync.startDone(error_t error) {}

	event void TimeSync.stopDone(error_t error) {}
}
