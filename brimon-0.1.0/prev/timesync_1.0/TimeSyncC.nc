#include "TimeSync.h"

configuration TimeSyncC {}

implementation {

	components TimeSyncM as App;
	components MainC, LedsC;
	components new Alarm32khz32C() as Timer0;
	components new Alarm32khz32C() as Timer1;
	components CC2420TimeSyncMessageC as TimeSyncMessage;  
	components ActiveMessageC;
	components CC2420PacketC;
	components CC2420RadioC;
	components new TimerMilliC();
  
	App.Boot -> MainC.Boot;
	App.Leds -> LedsC;
	App.Timer0 -> Timer0;
	App.Timer1 -> Timer1;
	App.AMControl -> ActiveMessageC;
	App.Receive -> TimeSyncMessage.Receive;
	App.TimeSyncAMSendI -> TimeSyncMessage.TimeSyncAMSend32khz[unique("asd")];
	App.TimeSyncPacket -> TimeSyncMessage;
	App.PacketTimeStamp32khz -> CC2420PacketC;
	App.PacketTimeSyncOffset -> CC2420PacketC;
	App.CC2420PacketBody -> CC2420PacketC;
	
	App.RadioControl -> CC2420RadioC.SplitControl;
	
	App.Timer2 -> TimerMilliC;
	components HplCC2420InterruptsC as Interrupts;
	App.CaptureSFD -> Interrupts.CaptureSFD;

	components HplCC2420PinsC as Pins;
	App.CSN -> Pins.CSN;

	components new CC2420SpiC() as Spi;
	App.TXFIFO_RAM  -> Spi.TXFIFO_RAM;

}

