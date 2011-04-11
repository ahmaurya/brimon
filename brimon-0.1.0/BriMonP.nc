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

module BriMonP
{
  uses {
    interface Boot;
    interface Timer<TMilli>;
    interface Sample;
    interface Leds;
  }
}
implementation
{
  event void Boot.booted() {
    call Timer.startPeriodic(SAMPLE_INTERVAL);
  }

  event void Timer.fired() {
    //call Leds.led0On();
    call Sample.sample();
  }

  event void Sample.sampled(error_t error) {
    //call Leds.led0Off();
    //call Leds.led1On();
    //call Summary.summarize();
    //call Timer.startOneShot(SAMPLE_INTERVAL);
    //if(error == SUCCESS)
	//	call Leds.set(7);
  }
}
