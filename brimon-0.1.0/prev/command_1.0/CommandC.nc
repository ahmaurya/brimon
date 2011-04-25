/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

/**
 * provides the Command interface. Should be used to call other functions
 * like timesync, routing, sensing, collection, transfer.
 */

configuration CommandC
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
	components CommandP;
	Command = CommandP;
	CommandP.Routing = Routing;
	//CommandP.TimeSync = TimeSync;
	CommandP.Sense = Sense;
	//CommandP.Collect = Collect;
	//CommandP.Transfer = Transfer;
}
