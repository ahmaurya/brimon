configuration LoggerC {
  provides interface Logger;
  provides interface Alarm<T32khz,uint32_t> as TransportTimer_proxy;
}
implementation {
  components LoggerM;
  Logger=LoggerM;
  components LoggerM as App, LedsC, MainC;
  components SerialActiveMessageC as AM;
  App.Control -> AM;
  components ActiveMessageC;
  App.PacketAcknowledgements->ActiveMessageC;

  components  new Alarm32khz32C();
  App.Alarm -> Alarm32khz32C;
  TransportTimer_proxy = Alarm32khz32C;


  components new TimerMilliC() as Timer1;
  App.Timer1 -> Timer1 ;  

  components UserButtonC;
  App.UB -> UserButtonC;

  components  CC2420PacketC;
  App.CC2420PacketBody -> CC2420PacketC;

  App.Leds -> LedsC;
  

  components  CC2420CsmaC;
  App.RadioControl->CC2420CsmaC;

  components CC2420ControlC;
  App.CC2420Config -> CC2420ControlC;
  
  components SerialComC;
  App.SerialCom->SerialComC;



}
