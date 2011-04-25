/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya, Jeet Patani
 * Copyright (C) 2011 Abhinav Maurya, Jeet Patani
 */

/*
	This file provides list of interfaces used and provided by TimeSync
*/

#include "TimeSync.h"

configuration TimeSyncC {
	provides interface TimeSync;
}

implementation {
	components TimeSyncP as App;  	// It provides TimeSync Interface
	components LedsC;		// Ledc is used to control Leds
	components new Alarm32khz32C() as Timer0;	// Variout timers
	components new Alarm32khz32C() as Timer1;
	components CC2420TimeSyncMessageC as TimeSyncMessage;	// CC2420 Message
	components ActiveMessageC;
	components CC2420PacketC;
	components CC2420RadioC;	// CC2420 RadioC to control Radios
	components new TimerMilliC();
	components RoutingC;		// RoutingC to get informatio about routing

	TimeSync = App.TimeSync;
	App.Leds -> LedsC;
	App.Timer0 -> Timer0;
	App.Timer1 -> Timer1;

	App.AMControl -> ActiveMessageC;
	App.Receive -> TimeSyncMessage.Receive[AM_TIMESYNC_MSG];
	App.TimeSyncAMSendI -> TimeSyncMessage.TimeSyncAMSend32khz[AM_TIMESYNC_MSG];
	App.TimeSyncPacket -> TimeSyncMessage;
	App.PacketTimeStamp32khz -> CC2420PacketC;
	App.PacketTimeSyncOffset -> CC2420PacketC;
	App.CC2420PacketBody -> CC2420PacketC;
	App.RadioControl -> CC2420RadioC.SplitControl;
	App.Timer2 -> TimerMilliC;
	App.Routing -> RoutingC;

	components HplCC2420InterruptsC as Interrupts;	// CaptureSFD is used to capture SFD interrupt
	App.CaptureSFD -> Interrupts.CaptureSFD;

	components HplCC2420PinsC as Pins;
	App.CSN -> Pins.CSN;

	components new CC2420SpiC() as Spi;
	App.TXFIFO_RAM  -> Spi.TXFIFO_RAM;
}

