/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Jeet Patani
 * Copyright (C) 2011 Jeet Patani
 */
/*
This file specifies interfaces used and provided by Routing.
*/


#include "Routing.h"

configuration RoutingC {
	provides interface Routing;

}

implementation {
	components RoutingP as App, LedsC;
	components new AMSenderC(AM_ROUTING_MSG);	// AMSenderC is used for sending data
	components new AMReceiverC(AM_ROUTING_MSG);	// AMReceiverC is used for reception of packets.
	components new TimerMilliC() as Timer1;			// Various timers are used for different purpose.
	components new TimerMilliC() as Timer2;
	components new TimerMilliC() as Timer3;
	components new TimerMilliC() as Timer4;
	components ActiveMessageC;			// ActiveMessageC is used for creating message
	components CC2420ActiveMessageC;		// CC2420 related components are used for chip specific operations like
	components CC2420ControlC;			// enabling hardware acks, etc.

  	Routing = App.Routing;
	App.Receive -> AMReceiverC;
  	App.AMSend -> AMSenderC;
  	App.AMControl -> ActiveMessageC;
  	App.Leds -> LedsC;
	App.MilliTimer -> Timer1;
	App.GiveupTimer -> Timer2;
	App.Packet -> AMSenderC;
	App.Timer32KHz -> Timer3;
	App.SendTimer -> Timer4;
	App.CC2420Packet -> CC2420ActiveMessageC.CC2420Packet;
	App.CC2420Config -> CC2420ControlC.CC2420Config;
	App.PacketAcknowledgements -> CC2420ActiveMessageC.PacketAcknowledgements;

}

