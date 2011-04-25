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
 * the main configuration file of the application that
 * creates the components used by CommandAppP
 */

configuration CommandAppC {}

implementation
{
	components CommandAppP as App;
	components CommandC;
	components MainC;
	components LedsC;
	components SerialActiveMessageC;
	components new TimerMilliC();
	components new TimerMilliC() as WaitTimerC;

	App.Boot -> MainC.Boot;
	App.Leds -> LedsC;
	App.Timer -> TimerMilliC;
	App.WaitTimer -> WaitTimerC;
	App.Command -> CommandC;

	App.Control -> SerialActiveMessageC;
	App.Packet -> SerialActiveMessageC;
	App.AMSend -> SerialActiveMessageC.AMSend[AM_TEST_SERIAL_MSG];
	App.Receive -> SerialActiveMessageC.Receive[AM_TEST_SERIAL_MSG];

	App.Routing -> CommandC.Routing;
	App.TimeSync -> CommandC.TimeSync;
	App.Sense -> CommandC.Sense;
	App.Collect -> CommandC.Collect;
	App.SensorDataRead -> CommandC.SensorDataRead;
}
