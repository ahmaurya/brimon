interface Routing
{
	command void route();
	command int getNumNodes();
	command int getParent(int);
	command int8_t* getRoutingTree();
  
	event void routeDone(error_t error);
}
