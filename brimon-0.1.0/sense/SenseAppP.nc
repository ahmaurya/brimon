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

module SenseAppP
{
  uses {
    interface Boot;
    interface Leds;
    interface Timer<TMilli>;
    interface Sense;
    //interface SplitControl as AMControl;
  }
}

implementation
{
  event void Boot.booted() {
    call Timer.startPeriodic(SAMPLE_INTERVAL);
    //call AMControl.start();
  }

  event void Timer.fired() {
    //call Leds.led0On();
    if(TOS_NODE_ID==1)
		call Sense.sense();
  }

  event void Sense.senseDone(error_t error) {
    //call Leds.led0Off();
    //call Leds.led1On();
    //call Summary.summarize();
    //call Timer.startOneShot(SAMPLE_INTERVAL);
    //if(error == SUCCESS)
	//	call Leds.set(7);
  }

  /*event void AMControl.startDone(error_t err){
	if (err != SUCCESS){
		call AMControl.start();
	}
  }*/

  //event void AMControl.stopDone(error_t err) {}
}
