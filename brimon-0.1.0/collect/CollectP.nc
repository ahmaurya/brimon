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
	This file provides detailed implementation of Data Transfer.
*/
#include "Collect.h"

module CollectP {
	provides interface Collect;
  uses {
		interface Leds;
    		interface Receive;
		interface AMSend;
    		interface Timer<TMilli> as MilliTimer;
		interface Timer<TMilli> as ReadTimer;
		interface Timer<TMilli> as WriteTimer;
		interface Timer<TMilli> as SeekTimer;
		interface Timer<TMilli> as SendTimer;
		interface Timer<TMilli> as GiveupTimer;
    		interface SplitControl as AMControl;
    		interface Packet;
		interface CC2420Packet;
		interface Random;
		interface ParameterInit<uint16_t> as SeedInit;
		interface PacketAcknowledgements;
		interface CC2420Config;
		interface LogRead;
		interface LogWrite;
		interface Routing;
	}
}

implementation {
	message_t packet[QUEUE_LENGTH];		// Packet Queue
	bool locked=FALSE;			// Lock variable to indicate whether radio busy or not.
	uint16_t seq=1;				// Local sequence number variable.
//	int8_t *tree_parent;			// Routing tree. It will be initialized using Routing Inerface
	int8_t tree_parent[3]={0,1,2};
	int msg_dest[QUEUE_LENGTH];		// Packet Queue
	int msg_length[QUEUE_LENGTH];		// Packet Queue
	bool ack_req[QUEUE_LENGTH];		// Packet Queue
	int send_req = 0;			// Packet Queue Pointer
	int send_done = 0;			// Packet Queue Pointer
	uint16_t data[BLK_SIZE];		// Array to store data before writing to flash or forwarding to parent.
	int data_pointer = 0;			// Pointer for the array data
	int data_sent = 0;			// Variable that keeps track of number of bytes sent
	int next_node;				// This variable stores which node to ask for data next.
	int block_req;				// This variable stores which block to ask for data next.
	int packet_map[TOTAL_PKT];		// This array stores NACK which will be transmitted at the end of reception of block.
	uint16_t offset;			// Offset variable for Flash offset.
	uint16_t sense_offset;			// Variable which will store offset where data is sensed. (Used by nodes other than head)
	uint16_t transfer_offset;		// This will store offset from where to transmit data. (used by head node)
	int buffer[BUFFER_SIZE];		// Buffer for reading-writing from flash.
	int current_block = 0;			// Keeps the track of which block is being transmitted currently.
	int retcnt = 0;				// Keeps track of retransmission count of each packet.


	void send_data(int);		// This function will send data. Which block is to be sent is passed as an argument.
	void ask_for_data(int,int);	// This function will send data request. It will use node id and block id as arguments.
	void send_data_packet(int,int);	// This function is used to send specific data packet.
	void send_transfer_done(void);	// This function will send data transfer done packet.

/*
	command collect first gets routing tree from routing interface and then erases the log.
	This will be called by Head node only.
	It will start asking for data when eraseDone even will be signaled.
*/


async command void Collect.collect(){
//	atomic tree_parent = call Routing.getRoutingTree();
	if(TOS_NODE_ID == 1){	// Erase the log if it is head node. Transfer will start at the end of erase.
		call LogWrite.erase();
	}
}

// This timer will be fired at the end of some threshold time of data request transmission if it does not receive data.

event void MilliTimer.fired() {
	if(TOS_NODE_ID == 1){	// Ask for same block again.
		ask_for_data(next_node,(block_req-1));
	}
}

event void SendTimer.fired(){
	if(locked){
		// Do nothing if it is locked.
	}
	else {
		// Request for hardware ack.
		if(call PacketAcknowledgements.requestAck(&packet[send_done]) == SUCCESS){
			ack_req[send_done]=TRUE;
		}
		else{
			ack_req[send_done]=FALSE;
		}
		// Send next packet from queue.
		if (call AMSend.send(msg_dest[send_done], &packet[send_done], msg_length[send_done]) == SUCCESS) {
			locked = TRUE;
		}
		else{
			call SendTimer.startOneShot(INTER_PKT_TIME);
		}

	}

}

// Giveup data transfer after some time.
event void GiveupTimer.fired(){
	signal Collect.collectDone(FAIL);
}

/*
	Receive event is signaled on reception of packet.It will act according to the packet received.

*/
event message_t* Receive.receive(message_t* bufPtr,void* payload, uint8_t len) {


	int i;
	if(!(call GiveupTimer.isRunning())){
		call GiveupTimer.startOneShot(GIVE_UP_TRANSFER);
	}
    	if (len == sizeof(ask_data_msg_t)){	// If message asking for data is received
		ask_data_msg_t* rcm = (ask_data_msg_t*)payload;
		if(rcm->type == DATA_REQUEST_MSG){
			if(rcm->dest == TOS_NODE_ID){ // If it is destination node, start sending data block requested.
				current_block = rcm->block;	// Set current_block to the requested block
				data_pointer = 0;		// Set data_pointer to beginning of the block.
				if((call LogRead.seek(sense_offset + (rcm->block*BLK_SIZE))) == SUCCESS){
					// Seek to the beginning of the block
				}
				else{
					// Retry if unable to send.
					call SeekTimer.startOneShot(INTER_LOG_TIME);
				}
			}
			else{
				// If not destination node, forward the packet to all children nodes.
		//		atomic tree_parent = call Routing.getRoutingTree();	// Get routing tree.
				for(i=0;i<NODES;i++){
					atomic if(tree_parent[i] == TOS_NODE_ID){
						ask_data_msg_t* fwd_pkt = (ask_data_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(ask_data_msg_t));

	     					if (fwd_pkt == NULL) {
							return bufPtr;
	      					}

						// Copy the packet fields.
						fwd_pkt->type = rcm->type;
						fwd_pkt->block = rcm->block;
						fwd_pkt->src = TOS_NODE_ID;
						fwd_pkt->dest = rcm->dest;

						// Put the packet on the queue.
						msg_dest[send_req]=i+1;
						msg_length[send_req]=sizeof(ask_data_msg_t);

						// Reset the pacekt map.
						for(i=0;i<TOTAL_PKT;i++){
							packet_map[i]=0;
						}
						// Send packet
						send_req++;
						if(send_req == QUEUE_LENGTH){
							send_req=0;
						}
						call SendTimer.startOneShot(INTER_PKT_TIME);

					}
				}
			}
		}
		else if(rcm->type == ACK_REQUEST_MSG){
			// If it is request for ack, send the NACK array.
			ack_msg_t* ack_pkt = (ack_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(ack_msg_t));
			if (ack_pkt == NULL) {
				return bufPtr;
			}

			// Set fields of packet.
			ack_pkt->type = ACK_REPLY_MSG;
			ack_pkt->src = TOS_NODE_ID;
			ack_pkt->block = rcm->block;
			ack_pkt->dest = rcm->src;
			for(i=0;i<TOTAL_PKT;i++){
				ack_pkt->ack[i] = packet_map[i];
			}

			// Put the packet on queue.
			msg_dest[send_req]=ack_pkt->dest;
			msg_length[send_req]=sizeof(ack_msg_t);

			// Call the timer to send the packet.
			send_req++;
			if(send_req == QUEUE_LENGTH){
				send_req=0;
			}

			call SendTimer.startOneShot(INTER_PKT_TIME);

		}
		else if(rcm->type == BLK_SUCCESS){
			// Block is successfully transfered from one node to another node.
			if(TOS_NODE_ID != 1){
				// Forward the block to parent node.
				send_data(rcm->block);
			}
			else if(TOS_NODE_ID == 1){

				// If head node, request for next block.
				call MilliTimer.stop();	// Stop the timer.
				data_pointer = 0;	// Reset the pointer
				for(i=0;i<BUFFER_SIZE;i++){
					buffer[i]=data[data_pointer];
					data_pointer++;
				}
				if((call LogWrite.append(buffer,sizeof(buffer))) == SUCCESS){
					// Append is successful.
				}
				else{
					// Retry if unable to perform append operation.
					call WriteTimer.startOneShot(INTER_LOG_TIME);
				}
			}
		}
		else if(rcm->type == TRANSFER_DONE){

			// If message received indicating end of data transfer, signal the collectDone event and send
			// the message to all children nodes.
			signal Collect.collectDone(SUCCESS);
			call GiveupTimer.stop();
			send_transfer_done();
		}
	    	return bufPtr;
	}
    	else if(len == sizeof(data_msg_t)){ // tree nessage

		data_msg_t* rcm = (data_msg_t*)payload;
		if(rcm->type == DATA_MSG_TYPE){
		// Data Packet Received. Store in data array.
			if(TOS_NODE_ID == 1){
				// Store the packet in array.
				for(i=0;i<PKT_SIZE;i++){
					data[((seq*PKT_SIZE)+i)]=rcm->data[i];
				}
				// Reset the timer.
				call MilliTimer.stop();
				call MilliTimer.startOneShot(TIME_OUT_REQ);
				packet_map[rcm->seq] = 1;	// Mark the packet as received.
			}
			else{
				// If not head node, then store the data in array data.
				for(i=0;i<PKT_SIZE;i++){
					data[((seq*PKT_SIZE)+i)]=rcm->data[i];
				}
				packet_map[rcm->seq] = 1;
			}
		}
		return bufPtr;
	}

	else if(len == sizeof(ack_msg_t)){ // tree nessage
		// If ack message is received, send the missing packets.
		ack_msg_t* rcm = (ack_msg_t*)payload;
		int retranscnt=0;

		if(rcm->type == ACK_REPLY_MSG){ // Received Packet Map. Retransmit lost packets.
			if(TOS_NODE_ID == rcm->dest){
				for(i=0;i<TOTAL_PKT;i++){
					if(rcm->ack[i] == 0){
						// Retransmit the missing packets.
						send_data_packet(rcm->block,i);
						retranscnt++;
					}
				}

				if(retranscnt==0){
					// If all packets are transfered successfully, send success messsage.
					ask_data_msg_t* fwd_pkt = (ask_data_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(ask_data_msg_t));
					if (fwd_pkt == NULL) {
						return bufPtr;
					}

					// Set the packet fields
					fwd_pkt->type = BLK_SUCCESS;
					fwd_pkt->block = rcm->block;
					fwd_pkt->src=TOS_NODE_ID;
					atomic fwd_pkt->dest = tree_parent[(TOS_NODE_ID - 1)];

					// Put the packet on the queue
					msg_dest[send_req]=fwd_pkt->dest;
					msg_length[send_req]=sizeof(ask_data_msg_t);

					send_req++;
					if(send_req == QUEUE_LENGTH){
						send_req=0;
					}
					// Send the packet
					call SendTimer.startOneShot(INTER_PKT_TIME);

				}
				else{
					// If all packets were not successfully transfered, send ack request again after transmitting few packets.

					ask_data_msg_t* fwd_pkt = (ask_data_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(ask_data_msg_t));
					if (fwd_pkt == NULL) {
						return bufPtr;
					}

					// Set the fields
					fwd_pkt->type = ACK_REQUEST_MSG;
					fwd_pkt->block = rcm->block;
					fwd_pkt->src=TOS_NODE_ID;
					atomic fwd_pkt->dest = tree_parent[(TOS_NODE_ID - 1)];

					// Put the message on the queue.
					msg_dest[send_req]=fwd_pkt->dest;
					msg_length[send_req]=sizeof(ask_data_msg_t);

					send_req++;
					if(send_req == QUEUE_LENGTH){
						send_req=0;
					}

					// Send the packet.
					call SendTimer.startOneShot(INTER_PKT_TIME);

				}
			}
		}
		return bufPtr;

	}
	else{
		// Return if any other message is received.
		return bufPtr;
	}
}

/*
	sendDone will be called when either hardware ack is received or it is timed our in case ack is requested.
*/

event void AMSend.sendDone(message_t* bufPtr, error_t error) {
	if (&packet[send_done] == bufPtr) {	// Make sure that event is triggered by the packet which was sent last.
		if(ack_req[send_done]){    // Check whether the ack was requested or not.

			if(call PacketAcknowledgements.wasAcked(&packet[send_done]) == TRUE){

				// Transmit next packet if ack is received.
				atomic locked = FALSE;
				atomic send_done++;
				retcnt=0;
				if(send_done == QUEUE_LENGTH){
					atomic send_done=0;
				}

				if(send_req != send_done){
					call SendTimer.startOneShot(INTER_PKT_TIME);
				}
			}
			else{
				// Retransmit the packet if ack was not received.
				atomic locked=FALSE;
				retcnt++;

				// Give up after some retransmissions.
				if(retcnt >=GIVE_UP){
					send_done++;
					if(send_done == QUEUE_LENGTH){
						send_done=0;
					}
					retcnt=0;
				}
				if(send_req != send_done){
					call SendTimer.startOneShot(INTER_PKT_TIME);
				}

			}
		}
		else{
			// If ack was not requested, send next packet.
			atomic locked = FALSE;
			retcnt=0;
			if(send_req != send_done){
				call SendTimer.startOneShot(INTER_PKT_TIME);
			}

		}

    	}
}

void send_data(int block){
	// This function assumes that data of the block is available in the array called 'data'

	int i;
	// Reset counters.
	data_sent=0;
	seq=0;

	while(data_sent < BLK_SIZE){
		data_msg_t* data_pkt = (data_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(data_msg_t));
      		if (data_pkt == NULL) {
			return;
      		}

		// Set the packet fields.
		data_pkt->type = DATA_MSG_TYPE;
		data_pkt->src=TOS_NODE_ID;
		data_pkt->block=block;
		data_pkt->seq=seq;
		atomic data_pkt->dest = tree_parent[(TOS_NODE_ID - 1)];

		// Copy the data from the array data
		for(i=0;i<PKT_SIZE;i++){
			data_pkt->data[i] = data[data_sent];
			data_sent=data_sent + 1;
		}

		seq++;
		// Put the packet on the queue.
		msg_dest[send_req]=data_pkt->dest;
		msg_length[send_req]=sizeof(data_msg_t);
		call SendTimer.startOneShot(INTER_PKT_TIME);
		send_req++;
		if(send_req == QUEUE_LENGTH){
			send_req=0;
		}
	}

	if(seq == TOTAL_PKT){
		// Request NACK at the end of all packet transmission
		ask_data_msg_t* fwd_pkt = (ask_data_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(ask_data_msg_t));

		if (fwd_pkt == NULL) {
			return;
		}
		// Set the packet fields
		fwd_pkt->type = ACK_REQUEST_MSG;
		fwd_pkt->block = block;
		fwd_pkt->src=TOS_NODE_ID;
		atomic fwd_pkt->dest = tree_parent[(TOS_NODE_ID - 1)];

		// Put the packet on the queue.
		msg_dest[send_req]=fwd_pkt->dest;
		msg_length[send_req]=sizeof(ask_data_msg_t);

		call SendTimer.startOneShot(INTER_PKT_TIME);
		send_req++;
		if(send_req == QUEUE_LENGTH){
			send_req=0;
		}
	}
	return;
}

/*
	This function will send data transfer request.
*/

void ask_for_data(int dest,int block_id){
	int i;
	for(i=0;i<NODES;i++){	// Send the requset to all children nodes.
		atomic if(tree_parent[i] == TOS_NODE_ID){
			ask_data_msg_t* fwd_pkt = (ask_data_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(ask_data_msg_t));
			if (fwd_pkt == NULL) {
				return;
			}

			// Set the fields
			fwd_pkt->type = DATA_REQUEST_MSG;
			fwd_pkt->block = block_id;
			fwd_pkt->src=TOS_NODE_ID;
			fwd_pkt->dest = dest;

			// Put the packet on the queue
			msg_dest[send_req]=i+1;
			msg_length[send_req]=sizeof(ask_data_msg_t);

			call SendTimer.startOneShot(INTER_PKT_TIME);
			send_req++;
			if(send_req == QUEUE_LENGTH){
				send_req=0;
			}
		}
	}
	call MilliTimer.startOneShot(TIME_OUT_REQ);
}

/*
	This function will send data transfer done message.

*/
void send_transfer_done(void){
	int i;
	for(i=0;i<NODES;i++){
		atomic if(tree_parent[i] == TOS_NODE_ID){
			ask_data_msg_t* fwd_pkt = (ask_data_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(ask_data_msg_t));
			if (fwd_pkt == NULL) {
				return;
      			}

			// Set the fields
			fwd_pkt->type = TRANSFER_DONE;
			fwd_pkt->block = 0;
			fwd_pkt->src=TOS_NODE_ID;
			fwd_pkt->dest = 0;

			// Put on the queue
			msg_dest[send_req]=i+1;
			msg_length[send_req]=sizeof(ask_data_msg_t);

			call SendTimer.startOneShot(INTER_PKT_TIME);
			send_req++;
			if(send_req == QUEUE_LENGTH){
				send_req=0;
			}
		}
	}
	call MilliTimer.startOneShot(TIME_OUT_REQ);
}

void send_data_packet(int block,int seq_n){
	int i;
	data_msg_t* data_pkt = (data_msg_t*)call AMSend.getPayload(&packet[send_req], sizeof(data_msg_t));
	if (data_pkt == NULL) {
		return;
	}

	// Set the field.
	data_pkt->type = DATA_MSG_TYPE;
	data_pkt->src=TOS_NODE_ID;
	data_pkt->block=block;
	data_pkt->seq=seq_n;
	atomic data_pkt->dest = tree_parent[(TOS_NODE_ID - 1)];
	data_sent = PKT_SIZE*seq_n;

	// Get appropriate data from buffer
	for(i=0 ;i<PKT_SIZE;i++){
		data_pkt->data[i] = data[data_sent];
		data_sent=data_sent+1;
	}

	// Put the packet on the queue
	msg_dest[send_req]=data_pkt->dest;
	msg_length[send_req]=sizeof(data_msg_t);
	call SendTimer.startOneShot(INTER_PKT_TIME);
	send_req++;
	if(send_req == QUEUE_LENGTH){
		send_req=0;
	}
	return;
}


/*
	Events fired AMControl and CC2420.
*/
event void AMControl.startDone(error_t err){}
event void AMControl.stopDone(error_t err) {}
event void CC2420Config.syncDone(error_t error) {
	if(error == SUCCESS){
	}
	else{
		call CC2420Config.sync();
	}
}


// Timer for retry read operation if it failed

event void ReadTimer.fired(){
	if((call LogRead.read(buffer,sizeof(buffer))) == SUCCESS){
		// Do nothing if read has started successfully
	}
	else{
		// Retry if failed to start read operation.
		call ReadTimer.startOneShot(INTER_LOG_TIME);
	}
}

// Event to handle read done.
event void LogRead.readDone(void *buf, storage_len_t len, error_t error){
	int i,j=0;
	if(error == SUCCESS){
		// If read operation is successful, copy the buffer to data
		for(i=0;i<BUFFER_SIZE;i++){
			data[data_pointer]=buffer[i];
			data_pointer++;
			if(data_pointer >= BLK_SIZE){
				j=1;
				break;
			}
		}
		if(j==0){ // If entire block is not read, read further.
			if((call LogRead.read(buffer,sizeof(buffer))) == SUCCESS){
				// Do nothing if read operation has started successfully
			}
			else{
				// Retry after some time if it failed
				call ReadTimer.startOneShot(INTER_LOG_TIME);
			}
		}
		else{
			// Start sending data if entire block has been sent.
			send_data(current_block);
		}
	}
}

// Timer to retry seek operation if it failed
event void SeekTimer.fired(){
	if((call LogRead.seek(sense_offset+(current_block*BLK_SIZE)) == SUCCESS)){
		// Do nothing if seek successfully started
	}
	else{
		// Retry after some time if it failed to start
		call SeekTimer.startOneShot(INTER_LOG_TIME);
	}
}

// Event to handle end of seek operation
event void LogRead.seekDone(error_t error){
	if(error == SUCCESS){
		if((call LogRead.read(buffer,sizeof(buffer))) == SUCCESS){
			// Do nothing if read was started successfully
		}
		else{
			// Retry after some time if failed
			call ReadTimer.startOneShot(INTER_LOG_TIME);
		}
	}
	else{
		call SeekTimer.startOneShot(INTER_LOG_TIME);
		if((call LogRead.seek(sense_offset+(current_block*BLK_SIZE)) == SUCCESS)){
		}
		else{
			// Retry after some time if failed
			call SeekTimer.startOneShot(INTER_LOG_TIME);
		}
	}
}

// Timer to handle retry of append operation
event void WriteTimer.fired(){

	if((call LogWrite.append(buffer,sizeof(buffer))) == SUCCESS){
		// Do nothing if append started successfully
	}
	else{
		// Retry after some time if failed
		call WriteTimer.startOneShot(INTER_LOG_TIME);
	}
}

// Event signaled at the end of append operation.
event void LogWrite.appendDone(void *buf, storage_len_t len, bool recordsLost, error_t error) {
	int i,j=0;
	if(error == SUCCESS){
		for(i=0;i<BUFFER_SIZE;i++){
			// Copy data from data to buffer
			buffer[i]=data[data_pointer];
			data_pointer++;
			if(data_pointer >= BLK_SIZE){
				j=1;
				break;
			}
		}
		if(j==0){ // If there is more data to append, call append
			if((call LogWrite.append(buffer,sizeof(buffer))) == SUCCESS){
				// Do nothing if append started successfully
			}
			else{
				// Retry after some time if failed
				call WriteTimer.startOneShot(INTER_LOG_TIME);
			}
		}
		else{	// If entire block has been written on the log, ask for next block
			if(block_req < TOTAL_BLOCKS){
				// Ask for next block
				ask_for_data(next_node,block_req);
				block_req++;
			}
			else if(next_node < NODES){
				// If all blocks are received, ask to next node
				block_req=0;
				next_node++;
				ask_for_data(next_node,block_req);
				block_req++;
			}
			else{
				// If all blocks from all nodes has been received, signal collectDone and send transfer done messages.
				signal Collect.collectDone(SUCCESS);
				call GiveupTimer.stop();
				send_transfer_done();
			}
		}
	}
	else{
		// If append is not successful, retry
		if((call LogWrite.append(buffer,sizeof(buffer))) == SUCCESS){
			// Do nothing if append started successfully
		}
		else{
			// Retry after some time if failed
			call WriteTimer.startOneShot(INTER_LOG_TIME);
		}
	}
}

// Event signaled at the end of Log erase operation
event void LogWrite.eraseDone(error_t error){

	if(error == SUCCESS){ // If erase was successful, then start asking for data
		next_node=2;
		block_req=0;
		ask_for_data(next_node,block_req);
		block_req++;
		sense_offset = call LogWrite.currentOffset();
		if(!(call GiveupTimer.isRunning())){
			call GiveupTimer.startOneShot(GIVE_UP_TRANSFER);
		}
	}
}

event void LogWrite.syncDone(error_t error) {
}
event void Routing.routeDone(error_t error){
}

}
