/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "Serial.h"

configuration SerialAppC {}

implementation {
  components SerialAppP as App, LedsC, MainC;
  components SerialActiveMessageC as AM;
  components new TimerMilliC();

  App.Boot -> MainC.Boot;
  App.Control -> AM;
  App.Receive -> AM.Receive[AM_TEST_SERIAL_MSG];
  App.AMSend -> AM.AMSend[AM_TEST_SERIAL_MSG];
  App.Leds -> LedsC;
  App.MilliTimer -> TimerMilliC;
  App.Packet -> AM;
}
