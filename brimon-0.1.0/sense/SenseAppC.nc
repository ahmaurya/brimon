/*
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
  components CC2420RadioC;
  components new LogStorageC(VOLUME_SAMPLELOG, TRUE);
  //new BlockStorageC(VOLUME_SAMPLES_X) as BlockStorageXC,
  //new BlockStorageC(VOLUME_SAMPLES_Y) as BlockStorageYC,
  //new BlockStorageC(VOLUME_SAMPLES_Z) as BlockStorageZC,

  components new Msp430AxisXC() as AccelXStreamC;
  components new Msp430AxisYC() as AccelYStreamC;
  components new Msp430AxisZC() as AccelZStreamC;
  //components new AccelXStreamC();
  //components new Msp430InternalVoltageC() as AccelXStreamC;

  SenseAppP.Boot -> MainC;
  SenseAppP.Leds -> LedsC;
  SenseAppP.Timer -> Timer;
  SenseAppP.Sense -> SenseC;

  SenseC.Leds -> LedsC;
  SenseC.LogWrite -> LogStorageC;
  SenseC.RadioControl -> CC2420RadioC.SplitControl;
}
