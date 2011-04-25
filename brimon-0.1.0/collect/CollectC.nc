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
	This file provides list of interfaces provided and used by interface Transfer.
*/
#include "Collect.h"

configuration CollectC {
	provides interface Collect;		// It provides Transfer Interface.
	provides interface LogRead;
}

implementation {
	components CollectP as App, LedsC;		// Implementation is in CollectP.
	components new AMSenderC(AM_COLLECT_MSG);		// AMSenderC is used for sending packets.
	components new AMReceiverC(AM_COLLECT_MSG);	// AMReceiver is used for packet reception.
	components new TimerMilliC();			// TimerMilliC is used for multiple purpose.
	components ActiveMessageC;			// Those interfaces are used for message and CC2420 related configuration.
	components CC2420ActiveMessageC;
	components CC2420ControlC;
	components RoutingC;				// Routing Interface is used by Collect to get Routing Details.

  	Collect = App.Collect;
	App.Receive -> AMReceiverC;
  	App.AMSend -> AMSenderC;
  	App.AMControl -> ActiveMessageC;
  	App.Leds -> LedsC;
	App.Routing -> RoutingC;
	App.MilliTimer -> TimerMilliC;
	App.ReadTimer -> TimerMilliC;
	App.SeekTimer -> TimerMilliC;
	App.WriteTimer -> TimerMilliC;
	App.SendTimer -> TimerMilliC;
	App.GiveupTimer -> TimerMilliC;
	App.Packet -> AMSenderC;
	App.CC2420Packet -> CC2420ActiveMessageC.CC2420Packet;
	App.CC2420Config -> CC2420ControlC.CC2420Config;
	App.PacketAcknowledgements -> CC2420ActiveMessageC.PacketAcknowledgements;

	components new LogStorageC(VOLUME_COLLECTLOG, TRUE);	// LogStorageC is used for Flash Read-Write operations.
	App.LogRead -> LogStorageC;
	App.LogWrite -> LogStorageC;
	LogRead = LogStorageC.LogRead;

}

