/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "Sense.h"

configuration SenseAppC {}

implementation
{
  components SenseAppP;
  components SenseC;
  components MainC, LedsC;
  components new TimerMilliC() as Timer;
  components ActiveMessageC;

  components new Msp430AxisXC() as AccelXStreamC;
  components new Msp430AxisYC() as AccelYStreamC;
  components new Msp430AxisZC() as AccelZStreamC;

  SenseAppP.Boot -> MainC;
  SenseAppP.Leds -> LedsC;
  SenseAppP.Timer -> Timer;
  SenseAppP.Sense -> SenseC;
  //SenseAppP.AMControl -> ActiveMessageC;
}
