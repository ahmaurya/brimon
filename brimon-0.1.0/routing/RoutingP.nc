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
This file provides implementation of Routing interface
*/

#include "Routing.h"

module RoutingP {
	provides interface Routing;
  uses {
		interface Leds;
    		interface Receive;
		interface AMSend;
    		interface Timer<TMilli> as MilliTimer;
		interface Timer<TMilli> as GiveupTimer;
    		interface SplitControl as AMControl;
    		interface Packet;
		interface Timer<TMilli> as Timer32KHz;
		interface Timer<TMilli> as SendTimer;
		interface CC2420Packet;
		interface ParameterInit<uint16_t> as SeedInit;
		interface PacketAcknowledgements;
		interface CC2420Config;
	}
}


implementation {
	message_t packet[QUEUE_LENGTH];		// This is an array of packets. It is used as packet queue.
	bool locked=FALSE;			// Lock variable to indicate whether the radio is busy or not.
	int8_t tree_parent[NODES];		// This array is used to store routing tree. It stores parent of ith enelent on (i-1)th index.
	uint8_t hello_start = 0;		// This is a flog to indicate whether the node has started sending Hello Messages or not.
	int num_hello;				// This variable keeps count of number of hello messages sent.
	int avg_RSSI[NODES];			// This array is used to store link state of a node.
	int no_of_packets[NODES];		// This array keeps track of number of hello messages received from perticular message to average RSSI
	int tree_build=0;			// A flag to indicate whether the tree has been built or not.
	int msg_dest[QUEUE_LENGTH];		// This is also packet queue which keeps track of next hop.
	int msg_length[QUEUE_LENGTH];		// This is also packet queue which keeps track of message length.
	bool ack_req[QUEUE_LENGTH];		// This is also packet queue which keeps track of whether the ack is expected or not.
	int send_req = 0;			// Producer Pointer of packet queue.
	int send_done = 0;			// Consumer Pointer of packet queue.
	int bad_tree[NODES];			// This array keep track of bad links if good link is not available.
	int routing_started = 0;		// The flag indicating whether routing is started or not.
	int retcnt = 0;				// The variable which keeps track of retransmission count.


void start_routing(void); 	// This function initiates routing. It is called only by Head Node.
void send_link_state(int,int);	// This function sends link state to the parent node whenever asked by Head Node.
void build_sub_tree(int,int);	// This function askes newly joined nodes to send their link states
bool check_routing_done(void);	// This function checks whether the routing has been done or not.
void send_routing_done(int);	// This function sends routing done message with finale routing tree.



/*
	sendHello Function sends 10 hello messages.
	The messages are broadcasted. All neighbours listening to those messages will start transmitting hello messages if already not transmitted.
*/
void sendHello(void){
	atomic tree_build = 0;
	atomic routing_started = 0;
	call Leds.set(7);
	if(!(call GiveupTimer.isRunning())){
		call GiveupTimer.startOneShot(GIVE_UP_TIME);
	}
	atomic if(num_hello > 0){
		// Send NUM_HELLO Hello Packets
		atomic num_hello--;
		call Timer32KHz.startOneShot(HELLO_MSG_INTERVAL);
	}
	else if(TOS_NODE_ID == 1){
		atomic num_hello--;
		// Begin routing after all nodes have transmitted their Hello Packets
		call Timer32KHz.startOneShot(NODES*HELLO_MSG_INTERVAL*NUM_HELLO);
	}
}

/*
	This function is used to print the current routing tree. It is used for debugging purpose.
*/


/*
	command getNumNodes simply return number of nodes.
*/
command int Routing.getNumNodes(){
	return NODES;
}

/*
	command getParent returns the parent node of the specified node.
*/

command int Routing.getParent(int child){
	atomic return tree_parent[(child-1)];
}

/*
	getRoutingTree returns entire routing tree.
*/
command int8_t* Routing.getRoutingTree(){
	return tree_parent;
}


/*
	command route implements actual routing mechanism
*/
command void Routing.route()
{
	// Initialize all the variables.

	int i;
	for(i=0;i<NODES;i++){
		atomic no_of_packets[i]=0;
		atomic tree_parent[i]=0;
		atomic bad_tree[i]=0;
		atomic avg_RSSI[i]=-100;

	}
	atomic tree_build = 0;
	atomic num_hello = NUM_HELLO;
	atomic hello_start = 1;
	printf("Sending Hello!\n");
	printfflush();
	sendHello();

	// Enable Hardware Acknowledgements
	call CC2420Config.setAddressRecognition(TRUE,TRUE);
	call CC2420Config.setAutoAck(TRUE,TRUE);
	call CC2420Config.sync();  //Synchronize new setting with Hardware

	call MilliTimer.startOneShot(MAX_HELLO_TIME); // This is the times which makes sure that routing begins within.
}


event void Timer32KHz.fired(){

	if(num_hello >=0 ){
		if(locked){ 		// Check whether the radio is busy or not.
			return;		// Here it will return if busy because 10 messages are sent. So if some messages are lost, it does not matter.
		}
		else {
  			routing_msg_t* rcm = (routing_msg_t*)call AMSend.getPayload(&packet[send_done], sizeof(routing_msg_t));

      			if (rcm == NULL) {
				return;
      			}

			// Hello Message. Set all the fields in the packet.

			rcm->type = HELLO_MESSAGE_TYPE;
			rcm->src=TOS_NODE_ID;
			if(call PacketAcknowledgements.noAck(&packet[send_done])){
				ack_req[send_done]=FALSE; // Request not to use ack.
			}
			msg_dest[send_done]=AM_BROADCAST_ADDR;	// Broadcast the packet.
			msg_length[send_done]=sizeof(routing_msg_t);
			if (call AMSend.send(msg_dest[send_done], &packet[send_done],msg_length[send_done]) == SUCCESS) {
				locked = TRUE;
			}
    		}
		sendHello(); // Send next hello message.
		}
	else if(TOS_NODE_ID == 1){
		start_routing(); 	// If NUM_HELLO Hello messages are sent and it is Head node then start routing
		atomic hello_start = 0;

	}
	else{
		hello_start = 0;	// Stop sending hello messages if not head node.
		printf("Hello done! %d \n",hello_start);
		printfflush();
	}
}

/*
	This is multipurpose timer. It will make sure initiation of routing. It will also periodically check whether routing is done or not.
	If not done, then it will replace missing links with available bad links and proceeds further.
*/

event void MilliTimer.fired() {
	int i,j;
	atomic if(TOS_NODE_ID == 1 && routing_started==0){
		start_routing(); // Start routing if not started already.
	}
	else if(tree_build == 0){
		if(check_routing_done()){		// Periodically check whether routing has been done or not.
			if(TOS_NODE_ID == 1){
				for(i=0;i<NODES;i++){
					atomic if(tree_parent[i] == TOS_NODE_ID){
						send_routing_done(i+1);		// Send routing done message if routing is done and it is head node.
					}
				}
			}
		}
		else{
			for(i=1;i<NODES;i++){
				atomic if(tree_parent[i]==0 && bad_tree[i]!=0){
					atomic tree_parent[i]=bad_tree[i];	// Replace messing links with bad links and proceed further.
					for(j=0;j<NODES;j++){
						atomic if(tree_parent[j]==TOS_NODE_ID){
							build_sub_tree(i+1,j+1); // Request for link state of newly joined nodes.
						}
					}
				}
			}
		}
	}
}

/*
	This timer will periodically send the messages if there are any message in the queue.
*/
event void SendTimer.fired(){

	if(locked){
	}
	else {

		// Ask for Hardware Acknowledgements

		if(call PacketAcknowledgements.requestAck(&packet[send_done]) == SUCCESS){
			ack_req[send_done]=TRUE;
		}
		else{
			ack_req[send_done]=FALSE;
		}
		if (call AMSend.send(msg_dest[send_done], &packet[send_done], msg_length[send_done]) == SUCCESS) {
			locked = TRUE;		// Wait for sendDone() event.
		}
		else{
			call SendTimer.startOneShot(INTR_PKT_INTERVAL); // Set the timer again if unable to send
		}

	}
}

event void GiveupTimer.fired(){
	call Leds.set(1);
	hello_start = 0;
	signal Routing.routeDone(FAIL);
}
/*
	Receive events handles all kind of packet reception.
	It will call appropriate function on reception of particular packets.
*/

event message_t* Receive.receive(message_t* bufPtr,void* payload, uint8_t len) {

	int rssi; // Local variables.
	int i;
	if (len == sizeof(routing_msg_t)){ // If Hello Message is received.

		routing_msg_t* rcm = (routing_msg_t*)payload;
		if(rcm->type == HELLO_MESSAGE_TYPE){  // Type is Hello Message.

			atomic if(hello_start == 0){ // Begin sending Hello packets if first Hello packet is received.
				atomic num_hello=NUM_HELLO;
				atomic hello_start=1;
				call Timer32KHz.startOneShot(HELLO_MSG_INTERVAL+10*TOS_NODE_ID); //This makes sure that every one does not transmit at same time.
			}

			rssi=(call CC2420Packet.getRssi(bufPtr))-45;	// Find the RSSI Value of received packet and calculate average.
			atomic avg_RSSI[(rcm->src-1)]=(((avg_RSSI[(rcm->src-1)]*no_of_packets[(rcm->src - 1)])+rssi)/(no_of_packets[(rcm->src - 1)]+1));
			no_of_packets[(rcm->src - 1)]++;
		    	return bufPtr;
		}
    	}
	else if(len == sizeof(tree_msg_t)){  // Tree Message is received.

		tree_msg_t* rcm = (tree_msg_t*)payload;
		for(i=0;i<NODES;i++){
			atomic tree_parent[i]=rcm->tree[i]; // Copy the tree from the message.
		}
		if(rcm->type == ASK_FOR_LINKS){
			if(rcm->dest == TOS_NODE_ID){
				send_link_state(TOS_NODE_ID,rcm->src); // Send Link state if it is destination.
			}
			else{
				if(!(check_routing_done())){	// Check whether routing is done or not. Discard if routing has been done.
					for(i=0;i<NODES;i++){
						atomic if(tree_parent[i] == TOS_NODE_ID){
							build_sub_tree(rcm->dest,i+1);	// Forward the message to children nodes to ask for their links.
						}
					}
				}
			}
		}
		else if(rcm->type==ROUTING_DONE_MSG){	// Routing has successfully done.
			atomic if(tree_build == 0){ // Make sure that event is signaled only once.
				hello_start = 0;
				call Leds.set(2);
				signal Routing.routeDone(SUCCESS);	//Signal routingDone event.
				call GiveupTimer.stop();
			}
			atomic if(tree_build == 0){
				for(i=0;i<NODES;i++){
					atomic if(tree_parent[i]== TOS_NODE_ID){
						send_routing_done(i+1);		// Send routing done message to all children nodes.
					}
				}
				atomic tree_build=1;
			}

		}
		return bufPtr;

	}
	else if (len == sizeof(link_msg_t)){ // Received a link message

		link_msg_t* lstat = (link_msg_t*)payload;
		if(lstat->type == LINK_MSG_TYPE){
			if(TOS_NODE_ID == 1){
				// If it is head node, then process the packet.
				int j;
				for(i=1;i<NODES;i++){
					atomic if(tree_parent[i] == 0 && lstat->link_rssi[i] > GOOD_LINK){
						atomic tree_parent[i]=lstat->src;	// Establish the link if it is good link.
						bad_tree[i]=TOS_NODE_ID;		// Update bad link with good link.
					}

					else if(tree_parent[i] == 0 && lstat->link_rssi[i] > BAD_LINK){
							bad_tree[i]=TOS_NODE_ID;	// If the link is bad, store it separately.
					}
				}
				if(!(check_routing_done())){	// Check whether routing has been done after processing the packet.
					for(i=0;i<NODES;i++){
						// Ask for the link state of the children of newly added node.
						atomic if(tree_parent[i]==lstat->src){
							for(j=0;j<NODES;j++){
								atomic if(tree_parent[j]==TOS_NODE_ID){
									build_sub_tree(i+1,j+1); // Ask for link state.
								}
							}
						}
					}
				}
			}
			else if (lstat->dest == TOS_NODE_ID){	// If it id the destination for the link message and not Head Node,
								// Forward the packet to parent node

				link_msg_t* l_stat = (link_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(link_msg_t));

      				if (l_stat == NULL) {
					return bufPtr;
      				}

				// Set the fields of the packet
				l_stat->type = lstat->type;
				l_stat->src=lstat->src; // Do not modify the source.
				atomic l_stat->dest = tree_parent[TOS_NODE_ID-1];
				for(i=0;i<NODES;i++){
					l_stat->link_rssi[i]=lstat->link_rssi[i];
				}

				// Put the packet in the queue.
				atomic msg_dest[send_req]=l_stat->dest;
				atomic msg_length[send_req]=sizeof(link_msg_t);
				atomic send_req++;
				call SendTimer.startOneShot(INTR_PKT_INTERVAL);

				atomic if(send_req == QUEUE_LENGTH ){	// Put the pointer to zero if reached at the end of the queue.
					atomic send_req = 0;
				}
			}
			return bufPtr;
		}
    	}
	else{	// In case any garbage message is received.
		return bufPtr;
	}
}

/*
	sendDone event is called when either acknowledgement is received (if requested) or when it times out.
	It will check for acknowledgement and retransmit the packet if acknowledgement is not received
*/

event void AMSend.sendDone(message_t* bufPtr, error_t error) {
	if (&packet[send_done] == bufPtr) {	// Ensure that event is triggered only for the packet which was sent.
		atomic if(ack_req[send_done]){      	// Check whether acknowledgemet was requested or not
			if(call PacketAcknowledgements.wasAcked(&packet[send_done]) == TRUE){
				// If ack is received, transmit nexr packet.
				atomic locked = FALSE;
				send_done++;
				retcnt=0;
				if(send_done == QUEUE_LENGTH){
					send_done=0;
				}
				atomic if(send_req != send_done){
					call SendTimer.startOneShot(INTR_PKT_INTERVAL);
				}
			}
			else{
				// If ack is not received and it timed out, retransmit the packet.

				atomic locked=FALSE;
				retcnt++;
				if(retcnt >=GIVE_UP){
					send_done++;
					if(send_done == QUEUE_LENGTH){
						send_done=0;
					}
					retcnt=0;
				}
				atomic if(send_req != send_done){
				call SendTimer.startOneShot(INTR_PKT_INTERVAL);
				}
			}
		}
		else{
			// Send next packet if ack was not requested.

			atomic locked = FALSE;
			retcnt=0;
			atomic if(send_req != send_done){
				call SendTimer.startOneShot(INTR_PKT_INTERVAL);
			}

		}

	}
}

/*
	This function initiates routing. It is executed by only head node.
	It will process hello messages received by the head node. And ask children of head node to send their link state.
*/

void start_routing(void){
	int i;
	for(i=0;i<NODES;i++){
		if(avg_RSSI[i] > GOOD_LINK){	// Add the node in the tree if link quality is good.
			atomic tree_parent[i]=TOS_NODE_ID;
			bad_tree[i]=TOS_NODE_ID;
		}
		else if(avg_RSSI[i] > BAD_LINK){
			bad_tree[i]=TOS_NODE_ID; // Store the link as bad link for further use.
		}
	}
	call MilliTimer.startPeriodic(PERIODIC_TIMER);
	routing_started=1;
	if(!(check_routing_done())){
		for(i=0;i<NODES;i++){
			atomic if(tree_parent[i]==1){
				build_sub_tree(i+1,i+1); // Ask for link messages to child node.

			}
		}
	}
}

/*
	This function will ask newly connected nodes to send their link packets.
*/

void build_sub_tree(int destination,int next_hop){
	int i;

	// Ask for link states to newly connected nodes.
	tree_msg_t* rcm = (tree_msg_t*) call AMSend.getPayload(&packet[send_req], sizeof(tree_msg_t));

	if (rcm == NULL) {
		return;
	}

	// Set the message fields

	rcm->type = ASK_FOR_LINKS;
	rcm->src=TOS_NODE_ID;
	rcm->dest = destination;
	for(i=0;i<NODES;i++){ // Put the current tree.
		atomic rcm->tree[i]=tree_parent[i];
	}
	atomic msg_dest[send_req]=next_hop;	// Put the message on queue.
	atomic msg_length[send_req]=sizeof(tree_msg_t);

	call SendTimer.startOneShot(INTR_PKT_INTERVAL);
	atomic send_req++;
	atomic if(send_req == QUEUE_LENGTH){
		atomic send_req=0;
	}
}

/*
	This function will send the link state to specific destination.
*/

void send_link_state(int src,int dest){

		// Send link state to the destination.

		int i;
		link_msg_t* lstat =  (link_msg_t*) call AMSend.getPayload(&packet[send_req], sizeof(link_msg_t));

		if (lstat == NULL) {
			return;
		}

		// Put the fields in the message.
		lstat->type = LINK_MSG_TYPE;
		lstat->src = src;
		lstat->dest = dest;

		for(i=0;i<NODES;i++){
			lstat->link_rssi[i]=avg_RSSI[i];
		}

		// Put the message on the queue.

		atomic msg_dest[send_req]=dest;
		atomic msg_length[send_req]=sizeof(link_msg_t);

		call SendTimer.startOneShot(INTR_PKT_INTERVAL);
		atomic send_req++;
		atomic if(send_req == QUEUE_LENGTH){
			atomic send_req=0;
		}
}

/*
	This function checks whether routing has been done or not.
*/
bool check_routing_done(void){
	int i;
	for(i=1;i<NODES;i++){
		atomic if(tree_parent[i]==0){
			return FALSE; // Return if any one node has no parent.
		}
	}

	call MilliTimer.stop();		// Stop the periodic timer.
	atomic if(tree_build == 0){
		for(i=0;i<NODES;i++){
			atomic if(tree_parent[i] == TOS_NODE_ID){
				send_routing_done(i+1);		// Send routing done message to all childrens.
			}
		}
	}
	atomic if(tree_build == 0 && TOS_NODE_ID == 1){
		call Leds.set(2);
		signal Routing.routeDone(SUCCESS);	// Signal the routeDone event.
		call GiveupTimer.stop();
		atomic tree_build = 1;
	}

	return TRUE;
}

void send_routing_done(int dest){
	int i;

	tree_msg_t* rcm = (tree_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(tree_msg_t));

	if (rcm == NULL) {
		return;
	}

	// Set appropriate fields in the message.
	rcm->type = ROUTING_DONE_MSG;
	rcm->src=TOS_NODE_ID;
	rcm->dest = dest;
	for(i=0;i<NODES;i++){
		atomic rcm->tree[i]=tree_parent[i];
	}

	// Put the message on the queue.
	atomic msg_dest[send_req]=dest;
	atomic msg_length[send_req]=sizeof(tree_msg_t);
	atomic send_req++;
	atomic if(send_req == QUEUE_LENGTH){
		atomic send_req=0;
	}

	call SendTimer.startOneShot(INTR_PKT_INTERVAL);
}


/*
	These are some event triggered by the interfaces used.
	Mostly they are not used by this interface
*/

event void AMControl.startDone(error_t err){
}
event void AMControl.stopDone(error_t err) {
}
event void CC2420Config.syncDone(error_t error) {
	if(error == SUCCESS){
	}
	else{
		call CC2420Config.sync();
	}
}
}
