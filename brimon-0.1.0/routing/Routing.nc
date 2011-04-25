/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Jeet Patani
 * Copyright (C) 2011 Jeet Patani
 */

/* 

This file provides interface to perform routing.All commands and event provided by routing is specified below.
route() is used to initiate routing. Any node in the topology can initiate routing using route command.
getNumNodes() returns number of nodes in topology.
getParent() returns parent node of specified node.
getRoutingTree() returns entire routing tree as an integer array. Element at ith position indicates parend node of (i+1)th node.
routeDone() event is signaled when routing is done.

*/

interface Routing
{
	command void route();
	command int getNumNodes();
	command int getParent(int);
	command int8_t* getRoutingTree();
  
	event void routeDone(error_t error);
}
