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

configuration SenseC {
	provides interface Sense;

	uses interface Leds;
	uses interface LogWrite;
	uses interface SplitControl as RadioControl;
}

implementation
{
	components SenseP;

	components new TimerMilliC() as Timer;
	components new Msp430AxisXC() as AccelXStreamC;
	components new Msp430AxisYC() as AccelYStreamC;
	components new Msp430AxisZC() as AccelZStreamC;

	Sense = SenseP.Sense;

	SenseP.Leds = Leds;
	SenseP.LogWrite = LogWrite;
	SenseP.Timer -> Timer;
	SenseP.RadioControl = RadioControl;

	SenseP.AccelX -> AccelXStreamC;
	SenseP.AccelY -> AccelYStreamC;
	SenseP.AccelZ -> AccelZStreamC;

	SenseP.ResourceX -> AccelXStreamC;
	SenseP.ResourceY -> AccelYStreamC;
	SenseP.ResourceZ -> AccelZStreamC;
}
