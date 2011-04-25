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

#include "Sense.h"

configuration CommandC
{
	provides interface Command;
	//uses interface Routing;
	//uses interface TimeSync;
	provides interface Sense;
	provides interface LogRead;
	//uses interface Collect;
	//uses interface Transfer;
}

implementation
{
	components MainC;
	components CommandP;
	//CommandP.Boot -> MainC.Boot;
	MainC.SoftwareInit -> CommandP.Init;
	Command = CommandP;
	//CommandP.Routing = Routing;
	//CommandP.TimeSync = TimeSync;
	//CommandP.Sense = Sense;
	//CommandP.Collect = Collect;
	//CommandP.Transfer = Transfer;

	components LedsC;
	components new TimerMilliC() as Timer;
	components new TimerMilliC() as AppTimerC;
	components new Msp430AxisXC() as AccelXStreamC;
	components new Msp430AxisYC() as AccelYStreamC;
	components new Msp430AxisZC() as AccelZStreamC;
	components new LogStorageC(VOLUME_SAMPLELOG, TRUE);
	//components CC2420RadioC;

	components new AMSenderC(AM_SENSE_MSG);
	components new AMReceiverC(AM_SENSE_MSG);
	components new TimerMilliC();
	components ActiveMessageC;
	components CC2420ActiveMessageC;
	components CC2420ControlC;
	//components RoutingC;

  	//CommandP.Routing -> RoutingC;
	CommandP.Receive -> AMReceiverC;
  	CommandP.AMSend -> AMSenderC;
  	CommandP.AMControl -> ActiveMessageC;
	CommandP.SendTimer -> TimerMilliC;
	CommandP.AppTimer -> AppTimerC;
	CommandP.Packet -> AMSenderC;
	CommandP.CC2420Packet -> CC2420ActiveMessageC.CC2420Packet;
	CommandP.CC2420Config -> CC2420ControlC.CC2420Config;
	CommandP.Resource -> CC2420ControlC;
	CommandP.PacketAcknowledgements -> CC2420ActiveMessageC.PacketAcknowledgements;

	Sense = CommandP.Sense;
	LogRead = LogStorageC.LogRead;

	CommandP.Leds -> LedsC;
	CommandP.LogWrite -> LogStorageC;
	CommandP.Timer -> Timer;
	//SenseP.RadioControl -> CC2420RadioC;

	CommandP.AccelX -> AccelXStreamC;
	CommandP.AccelY -> AccelYStreamC;
	CommandP.AccelZ -> AccelZStreamC;

	CommandP.ResourceX -> AccelXStreamC;
	CommandP.ResourceY -> AccelYStreamC;
	CommandP.ResourceZ -> AccelZStreamC;
}
