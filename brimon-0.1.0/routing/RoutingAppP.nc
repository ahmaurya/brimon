/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Jeet Patani
 * Copyright (C) 2011 Jeet Patani
 */

#include "Routing.h"

module RoutingAppP {
  uses {
		interface Routing;
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
			if(TOS_NODE_ID == 1){
			call MilliTimer.startPeriodic(10000);	
			}
		}
		else{
			call AMControl.start();
		}
	}
	event void MilliTimer.fired() {
		call Routing.route();
	}

	event void AMControl.stopDone(error_t err) {}
	event void Routing.routeDone(error_t err){

		int nodes = call Routing.getNumNodes();
		int i;
		printf("NumNodes : %d\n",call Routing.getNumNodes());
		printf("Parent : %d\n",call Routing.getParent(TOS_NODE_ID));
		tree = call Routing.getRoutingTree();		
		printf("Tree : ");
		for(i=0;i<nodes;i++){
			printf("%d -- ",tree[i]);
		}
		printfflush();
	}
}
