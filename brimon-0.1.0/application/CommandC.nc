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

#include "Command.h"

configuration CommandC
{
	provides {
		interface Command;
		interface Routing;
		interface TimeSync;
		interface Sense;
		interface Collect;
		interface LogRead as SensorDataRead;
		interface LogRead as CollectDataRead;
	}
}

implementation
{
	/* common components */
	components MainC;
	components LedsC;
	components ActiveMessageC;
	components CC2420ActiveMessageC;
	components CC2420ControlC;
	components CC2420RadioC;
	components CommandP;
	components RoutingP;
	components TimeSyncP;
	components SenseP;
	components CollectP;

	/** sense code */
	//components SenseP;
	//components LedsC;
	components new TimerMilliC() as SenseTimer;
	components new TimerMilliC() as SenseTimerMilliC;

	components new Msp430AxisXC() as AccelXStreamC;
	components new Msp430AxisYC() as AccelYStreamC;
	components new Msp430AxisZC() as AccelZStreamC;
	components new LogStorageC(VOLUME_SAMPLELOG, TRUE) as SenseStorageC;

	components new AMSenderC(AM_SENSE_MSG) as SenseSenderC;
	components new AMReceiverC(AM_SENSE_MSG) as SenseReceiverC;
	//components ActiveMessageC;
	//components CC2420ActiveMessageC;
	//components CC2420ControlC;
	//components RoutingC;

	SenseP.AMControl -> ActiveMessageC;
  	SenseP.Routing -> RoutingP;
  	SenseP.Packet -> SenseSenderC;
  	SenseP.AMSend -> SenseSenderC;
  	SenseP.Receive -> SenseReceiverC;
	SenseP.SendTimer -> SenseTimerMilliC;

	//SenseP.CC2420Packet -> CC2420ActiveMessageC.CC2420Packet;
	//SenseP.CC2420Config -> CC2420ControlC.CC2420Config;
	//SenseP.PacketAcknowledgements -> CC2420ActiveMessageC.PacketAcknowledgements;

	//Sense = SenseP.Sense;
	//LogRead = SenseStorageC.LogRead;
	SenseP.Leds -> LedsC;
	SenseP.LogWrite -> SenseStorageC;
	SenseP.Timer -> SenseTimer;

	SenseP.AccelX -> AccelXStreamC;
	SenseP.AccelY -> AccelYStreamC;
	SenseP.AccelZ -> AccelZStreamC;
	SenseP.ResourceX -> AccelXStreamC;
	SenseP.ResourceY -> AccelYStreamC;
	SenseP.ResourceZ -> AccelZStreamC;

	/* routing code */
	//components RoutingP;
	//components LedsC;
	components new AMSenderC(AM_ROUTING_MSG) as RoutingSenderC;
	components new AMReceiverC(AM_ROUTING_MSG) as RoutingReceiverC;
	components new TimerMilliC() as Timer1;
	components new TimerMilliC() as Timer2;
	components new TimerMilliC() as Timer3;
	components new TimerMilliC() as Timer4;
	//components ActiveMessageC;
	//components CC2420ActiveMessageC;
	//components CC2420ControlC;

  	//Routing = RoutingP.Routing;
	RoutingP.Packet -> RoutingSenderC;
	RoutingP.AMSend -> RoutingSenderC;
	RoutingP.Receive -> RoutingReceiverC;
  	RoutingP.AMControl -> ActiveMessageC;
  	RoutingP.Leds -> LedsC;
	RoutingP.MilliTimer -> Timer1;
	RoutingP.GiveupTimer -> Timer2;
	RoutingP.Timer32KHz -> Timer3;
	RoutingP.SendTimer -> Timer4;
	RoutingP.CC2420Packet -> CC2420ActiveMessageC.CC2420Packet;
	RoutingP.CC2420Config -> CC2420ControlC.CC2420Config;
	RoutingP.PacketAcknowledgements -> CC2420ActiveMessageC.PacketAcknowledgements;

	/* collect code */
	//components CollectP;
	//components LedsC;
	components new AMSenderC(AM_COLLECT_MSG) as CollectSenderC;
	components new AMReceiverC(AM_COLLECT_MSG) as CollectReceiverC;
	components new TimerMilliC() as CollectTimerC;
	//components ActiveMessageC;
	//components CC2420ActiveMessageC;
	//components CC2420ControlC;
	//components RoutingC;

  	Collect = CollectP.Collect;
  	CollectP.Packet -> CollectSenderC;
  	CollectP.AMSend -> CollectSenderC;
  	CollectP.Receive -> CollectReceiverC;
  	CollectP.Leds -> LedsC;
	CollectP.Routing -> RoutingP;
	CollectP.MilliTimer -> CollectTimerC;
	CollectP.ReadTimer -> CollectTimerC;
	CollectP.SeekTimer -> CollectTimerC;
	CollectP.WriteTimer -> CollectTimerC;
	CollectP.SendTimer -> CollectTimerC;
	CollectP.GiveupTimer -> CollectTimerC;
	CollectP.PacketAcknowledgements -> CC2420ActiveMessageC.PacketAcknowledgements;

	components new LogStorageC(VOLUME_COLLECTLOG, TRUE) as CollectStorageC;
	CollectP.LogRead -> CollectStorageC;
	CollectP.LogWrite -> CollectStorageC;
	CollectDataRead = CollectStorageC.LogRead;

	/* timesync code */
	//components TimeSyncP;
	//components LedsC;
	components new Alarm32khz32C() as TSTimer0;
	components new Alarm32khz32C() as TSTimer1;
	components CC2420TimeSyncMessageC as TimeSyncMessage;
	//components ActiveMessageC;
	components CC2420PacketC;
	//components CC2420RadioC;
	components new TimerMilliC() as TSTimer;
	//components RoutingC;

	TimeSync = TimeSyncP.TimeSync;
	TimeSyncP.Leds -> LedsC;
	TimeSyncP.Timer0 -> TSTimer0;
	TimeSyncP.Timer1 -> TSTimer1;

	TimeSyncP.AMControl -> ActiveMessageC;
	TimeSyncP.Receive -> TimeSyncMessage.Receive[AM_TIMESYNC_MSG];
	TimeSyncP.TimeSyncAMSendI -> TimeSyncMessage.TimeSyncAMSend32khz[AM_TIMESYNC_MSG];
	TimeSyncP.TimeSyncPacket -> TimeSyncMessage;
	TimeSyncP.PacketTimeStamp32khz -> CC2420PacketC;
	TimeSyncP.PacketTimeSyncOffset -> CC2420PacketC;
	TimeSyncP.CC2420PacketBody -> CC2420PacketC;
	TimeSyncP.RadioControl -> CC2420RadioC.SplitControl;
	TimeSyncP.Timer2 -> TSTimer;
	TimeSyncP.Routing -> RoutingP;

	components HplCC2420InterruptsC as Interrupts;
	TimeSyncP.CaptureSFD -> Interrupts.CaptureSFD;

	components HplCC2420PinsC as Pins;
	TimeSyncP.CSN -> Pins.CSN;

	components new CC2420SpiC() as Spi;
	TimeSyncP.TXFIFO_RAM  -> Spi.TXFIFO_RAM;

	/* command code */
	MainC.SoftwareInit -> CommandP.Init;
	CommandP.AMControl -> ActiveMessageC;

	Command = CommandP;
	Routing = RoutingP;
	TimeSync = TimeSyncP;
	Sense = SenseP;
	Collect = CollectP;
	SensorDataRead = SenseStorageC.LogRead;

	CommandP.Routing -> RoutingP;
	CommandP.TimeSync -> TimeSyncP;
	CommandP.Sense -> SenseP;
	CommandP.Collect -> CollectP;

}
