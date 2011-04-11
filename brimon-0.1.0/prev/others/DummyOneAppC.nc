#include "DummyOne.h"

configuration DummyOneAppC {}

implementation {

  components MainC;
  components LedsC;
  components new TimerMilliC() as TimerC;
  components DummyOneC as AppC;

  AppC -> MainC.Boot;
  AppC.Leds -> LedsC;
  AppC.Timer -> TimerC;

}
