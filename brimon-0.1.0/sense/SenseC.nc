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

configuration SenseC
{
	provides interface Sense;
	provides interface LogRead;
}

implementation
{
	components SenseP;

	components LedsC;
	components new TimerMilliC() as Timer;
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
	components RoutingC;

  	SenseP.Routing -> RoutingC;
	SenseP.Receive -> AMReceiverC;
  	SenseP.AMSend -> AMSenderC;
  	SenseP.AMControl -> ActiveMessageC;
	SenseP.SendTimer -> TimerMilliC;
	SenseP.Packet -> AMSenderC;
	SenseP.CC2420Packet -> CC2420ActiveMessageC.CC2420Packet;
	SenseP.CC2420Config -> CC2420ControlC.CC2420Config;
	SenseP.PacketAcknowledgements -> CC2420ActiveMessageC.PacketAcknowledgements;

	Sense = SenseP.Sense;
	LogRead = LogStorageC.LogRead;

	SenseP.Leds -> LedsC;
	SenseP.LogWrite -> LogStorageC;
	SenseP.Timer -> Timer;
	//SenseP.RadioControl -> CC2420RadioC;

	SenseP.AccelX -> AccelXStreamC;
	SenseP.AccelY -> AccelYStreamC;
	SenseP.AccelZ -> AccelZStreamC;

	SenseP.ResourceX -> AccelXStreamC;
	SenseP.ResourceY -> AccelYStreamC;
	SenseP.ResourceZ -> AccelZStreamC;
}
