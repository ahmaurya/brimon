#include "DummyTwo.h"

configuration DummyTwoAppC {}

implementation {

  components MainC;
  components LedsC;
  components new TimerMilliC() as TimeC;
  components DummyTwoC as App;
  components new AMSenderC(AM_RADIO_COUNT_MSG);
  components new AMReceiverC(AM_RADIO_COUNT_MSG);
  components ActiveMessageC;

  App.Boot -> MainC.Boot;
  App.Leds -> LedsC.Leds;
  App.MilliTimer -> TimeC;
  App.AMSend -> AMSenderC;
  App.Packet -> AMSenderC;
  App.Receive -> AMReceiverC;
  App.AMControl -> ActiveMessageC;

}
