/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "BriMon.h"

configuration BriMonAppC {
	//provides interface Sense;
	//provides interface LogRead;
}

implementation
{
  components SenseP;
  components SamplerC;
  components MainC, LedsC;
  components new TimerMilliC() as Timer, new TimerMilliC() as TimerA;
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

  SenseP.Boot -> MainC;
  SenseP.Leds -> LedsC;
  SenseP.Timer -> TimerA;
  SenseP.Sample -> SamplerC;

  SamplerC.AccelX -> AccelXStreamC;
  SamplerC.AccelY -> AccelYStreamC;
  SamplerC.AccelZ -> AccelZStreamC;

  SamplerC.ResourceX -> AccelXStreamC;
  SamplerC.ResourceY -> AccelYStreamC;
  SamplerC.ResourceZ -> AccelZStreamC;

  SamplerC.LogWrite -> LogStorageC;
  SamplerC.Leds -> LedsC;
  SamplerC.Timer -> Timer;
  SamplerC.RadioControl -> CC2420RadioC.SplitControl;
}
