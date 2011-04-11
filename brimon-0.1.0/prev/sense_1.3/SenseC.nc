/*
 * Copyright (c) 2007-2009 Intel Corporation
 * All rights reserved.

 * This file is distributed under the terms in the attached INTEL-LICENS
 * file. If you do not find this file, a copy can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */

#include "Sense.h"

configuration SenseC {
	//provides interface Sense;
}

implementation
{
  components SenseP;
  components SummarizerC, SamplerC;
  components MainC, LedsC, new TimerMilliC(),
  new LogStorageC(VOLUME_SAMPLELOG, TRUE),
  new BlockStorageC(VOLUME_SAMPLES_X) as BlockStorageXC,
  new BlockStorageC(VOLUME_SAMPLES_Y) as BlockStorageYC,
  new BlockStorageC(VOLUME_SAMPLES_Z) as BlockStorageZC,

  new Msp430AxisXC() as AccelXStreamC,
  new Msp430AxisYC() as AccelYStreamC,
  new Msp430AxisZC() as AccelZStreamC;
  //new AccelXStreamC();
  //new Msp430InternalVoltageC() as AccelXStreamC;

  SenseP.Boot -> MainC;
  SenseP.Leds -> LedsC;
  SenseP.Timer -> TimerMilliC;
  SenseP.Summary -> SummarizerC;
  SenseP.Sample -> SamplerC;

  SamplerC.AccelX -> AccelXStreamC;
  SamplerC.AccelY -> AccelYStreamC;
  SamplerC.AccelZ -> AccelZStreamC;

  SamplerC.BlockReadX -> BlockStorageXC;
  SamplerC.BlockReadY -> BlockStorageYC;
  SamplerC.BlockReadZ -> BlockStorageZC;

  SamplerC.BlockWriteX -> BlockStorageXC;
  SamplerC.BlockWriteY -> BlockStorageYC;
  SamplerC.BlockWriteZ -> BlockStorageZC;

  SamplerC.LogWrite -> LogStorageC;
  SamplerC.Leds -> LedsC;
  SamplerC.Timer -> TimerMilliC;

  SummarizerC.BlockRead -> BlockStorageXC;
  SummarizerC.LogWrite -> LogStorageC;
}
