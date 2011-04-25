#include "Routing.h"

configuration RoutingC {
	provides interface Routing;

}

implementation {
	components RoutingP as App, LedsC;
	components new AMSenderC(AM_ROUTING_MSG);
	components new AMReceiverC(AM_ROUTING_MSG);
	components new TimerMilliC();
	components new Alarm32khz32C() as Timer32KHz;
	components new Alarm32khz32C() as SendTimer;
	components ActiveMessageC;
	components CC2420ActiveMessageC;
	components CC2420ControlC;
  
  	Routing = App.Routing;
	App.Receive -> AMReceiverC;
  	App.AMSend -> AMSenderC;
  	App.AMControl -> ActiveMessageC;
  	App.Leds -> LedsC;
	App.MilliTimer -> TimerMilliC;
	App.Packet -> AMSenderC;
	App.Timer32KHz -> Timer32KHz;
	App.SendTimer -> SendTimer;
	App.CC2420Packet -> CC2420ActiveMessageC.CC2420Packet;
	App.CC2420Config -> CC2420ControlC.CC2420Config;
	App.PacketAcknowledgements -> CC2420ActiveMessageC.PacketAcknowledgements;

}

