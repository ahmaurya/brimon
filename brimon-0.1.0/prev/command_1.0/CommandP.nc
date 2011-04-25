/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "Command.h"

/** provides the implementation for the commands of the Command interface
 * provided by CommandC. Include CommandC for using various functionalities
 */

module CommandP
{
	provides interface Command;
	uses interface Routing;
	//uses interface TimeSync;
	uses interface Sense;
	//uses interface Collect;
	//uses interface Transfer;
}

implementation
{
	/** a simple switch case for calling various functionalities
	 * as per the command to be executed
	 */
	command void Command.execCommand(uint16_t commType)
	{
		switch(commType)
		{
			case ROUTING: call Routing.route(); break;
			//case TIMESYNC_START: call TimeSync.start(); break;
			//case TIMESYNC_STOP: call TimeSync.stop(); break;
			case SENSE: call Sense.sense(); break;
			//case COLLECT: call Collect.collect(); break;
			//case TRANSFER: call Transfer.transfer(); break;
		}
	}

	/**
	 * On receiving routeDone, signal that the command has finished execution
	 * as per the Command interface
	 */
	event void Routing.routeDone(error_t error)
	{
		signal Command.execCommandDone(error, ROUTING);
	}

/*
	event void TimeSync.startDone(error_t error)
	{
		signal Command.execCommandDone(error, TIMESYNC_START);
	}

	event void TimeSync.stopDone(error_t error)
	{
		signal Command.execCommandDone(error, TIMESYNC_STOP);
	}
*/

	/**
	 * On receiving senseDone, signal that the command has finished execution
	 * as per the Command interface
	 */
	event void Sense.senseDone(error_t error)
	{
		signal Command.execCommandDone(error, SENSE);
	}

/*
	event void Collect.collectDone(error_t error)
	{
		signal Command.execCommandDone(error, COLLECT);
	}

	event void Transfer.transferDone(error_t error)
	{
		signal Command.execCommandDone(error, TRANSFER);
	}
*/
}
