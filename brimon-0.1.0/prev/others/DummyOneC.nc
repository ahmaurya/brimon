#include "Timer.h"
#include "DummyOne.h"

module DummyOneC @safe() {
  uses {
    interface Boot;
    interface Leds;
    interface Timer<TMilli> as Timer;
  }
}

implementation {

  uint16_t counter = 0;

  event void Boot.booted() {
    call Timer.startPeriodic(TIMER_PERIOD);
  }

  event void Timer.fired() {
    counter++;
    call Leds.set(counter);
  }

}
