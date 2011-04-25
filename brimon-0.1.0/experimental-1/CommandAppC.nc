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
	components CommandAppP as App, LedsC, MainC;
	components SerialActiveMessageC as AM;
	components ActiveMessageC;
	components new TimerMilliC();
	components new TimerMilliC() as WaitTimerC;
	components CommandC;
	components RoutingC;
	components TimeSyncC;
	components SenseC;
	components CollectC;
	//components TransferC;

	App.Boot -> MainC.Boot;
	App.Control -> AM;
	App.Receive -> AM.Receive[AM_TEST_SERIAL_MSG];
	App.AMSend -> AM.AMSend[AM_TEST_SERIAL_MSG];
	App.Leds -> LedsC;
	App.Packet -> AM;
	App.AMControl -> ActiveMessageC;
	App.Timer -> TimerMilliC;
	App.WaitTimer -> WaitTimerC;

	App.Command -> CommandC;
	App.Routing -> RoutingC;
	App.TimeSync -> TimeSyncC;
	App.Sense -> SenseC;
	App.Collect -> CollectC;
	//App.Transfer -> TransferC;
	App.SensorDataRead -> SenseC.LogRead;

	CommandC.Routing -> RoutingC;
	CommandC.TimeSync -> TimeSyncC;
	CommandC.Sense -> SenseC;
	CommandC.Collect -> CollectC;
	//CommandC.Transfer -> TransferC;
}
