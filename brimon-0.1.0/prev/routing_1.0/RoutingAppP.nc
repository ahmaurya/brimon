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
			call MilliTimer.startOneShot(10000);	
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
	}
}
