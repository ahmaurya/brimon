/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Jeet Patani
 * Copyright (C) 2011 Jeet Patani
 */

#include "Collect.h"

module CollectAppP {
  uses {
		interface Routing;
		interface Collect;
		interface Leds;
    		interface Timer<TMilli> as MilliTimer;
    		interface SplitControl as AMControl;
		interface Boot;
	}
}

implementation {
	int8_t *tree;
	event void Boot.booted()
	{
		call AMControl.start();
	}

	event void AMControl.startDone(error_t err)
	{
		if (err == SUCCESS){
		//	tree = call Routing.getRoutingTree();
			if(TOS_NODE_ID == 1){
			call MilliTimer.startOneShot(10000);
			}
		}
		else{
			call AMControl.start();
		}
	}
	event void MilliTimer.fired() {
		call Collect.collectData();
	}

	event void AMControl.stopDone(error_t err) {}
	event void Collect.collectDataDone(error_t err){

	}
	event void Routing.routeDone(error_t error){
	}

}
