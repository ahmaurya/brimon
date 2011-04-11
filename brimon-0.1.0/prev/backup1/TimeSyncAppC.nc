
# include<printf.h>

configuration TimeSyncAppC {}

implementation {

	components MainC,TimeSyncC as App, LedsC;
	components new Alarm32khz32C();
	components CC2420TimeSyncMessageC as TimeSyncM;  
	components ActiveMessageC;
	components CC2420PacketC;
	components new TimerMilliC() as Timer1;
	components new Alarm32khz32C() as Timer2;
	components CC2420RadioC;
  
	App.Boot -> MainC.Boot;
	App.Leds -> LedsC;
	App.Timer0 -> Alarm32khz32C;
	App.AMControl -> ActiveMessageC;
	App.Receive -> TimeSyncM.Receive;
	App.TimeSyncAMSendI -> TimeSyncM.TimeSyncAMSend32khz[unique("asd")];
	App.TimeSyncPacket -> TimeSyncM;
	App.PacketTimeStamp32khz -> CC2420PacketC;
	App.PacketTimeSyncOffset -> CC2420PacketC;
	App.CC2420PacketBody -> CC2420PacketC;
	
	App.Timer1 -> Timer1;
	App.Timer2 -> Timer2;
	components HplCC2420InterruptsC as Interrupts;
	App.CaptureSFD -> Interrupts.CaptureSFD;

	components HplCC2420PinsC as Pins;
	App.CSN -> Pins.CSN;


	components new CC2420SpiC() as Spi;
	App.TXFIFO_RAM  -> Spi.TXFIFO_RAM;
	
	App.RadioControl -> CC2420RadioC.SplitControl;

}

