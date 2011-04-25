/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya, Jeet Patani
 * Copyright (C) 2011 Abhinav Maurya, Jeet Patani
 */

/*
	This file provides implementation of TimeSync Interface.
*/

#include "TimeSync.h"

module TimeSyncP {
	provides interface TimeSync;
	uses {
		interface Leds;
		interface Alarm <T32khz,uint32_t> as Timer0;
		interface Alarm <T32khz,uint32_t> as Timer1;
		interface Timer<TMilli> as Timer2;
		interface TimeSyncAMSend<T32khz,uint32_t> as TimeSyncAMSendI;
		interface TimeSyncPacket<T32khz,uint32_t> ;
		interface Receive;
		interface SplitControl as AMControl;
		interface Packet;
		interface PacketTimeStamp <T32khz, uint32_t> as PacketTimeStamp32khz;
		interface GpioCapture as CaptureSFD;
		interface GeneralIO as CSN;
		interface CC2420Ram as TXFIFO_RAM;
		interface PacketTimeSyncOffset;
		interface CC2420PacketBody;
		interface SplitControl as RadioControl;
		interface Routing;
	}
}

implementation {
	message_t packet;		// Packet
	bool locked;			// Variable to indicate lock on radio
	uint32_t Tw=TWK, Tsl=TSL;	// Wakeup and sleep interval
	uint8_t flag=1;			// Flag used to indicate whether the radio is up or not.
	uint32_t timeSyncOffset = 0;	// This variable stores timesync offset.
	uint32_t globalTime = 0;	// This variable stores global time.
	uint32_t wakeSyncOffset = 0;	// This variable stores next wakeup offset
//	int8_t* tree_parent;		// This array stores routing tree
	uint8_t synchronized = 0;	// This flag indicates whether the node is synchronized with the parent or not
	uint32_t dutyCycleOffset=0;	// This variable stores offset of dutycycle.
	uint8_t ifTimeSyncStop=STOP_CYCLE;	// The variable to indicate whether to duty cycle or not.
	int8_t stop_count = STOP_DUTY_CYCLE;	// The variable indicating number of transmission of packet
	inline void* getFooter(message_t* msg);
	uint8_t payloadLength(message_t* msg);
	int8_t tree_parent[3]={0,1,2};
void initializeParameters() {
	// Initialize all the parameters
	locked=FALSE;
	Tw=TWK;
	Tsl=TSL;
	atomic flag=1;
	timeSyncOffset = 0;
	globalTime = 0;
	wakeSyncOffset = 0;
	synchronized = 0;
	dutyCycleOffset=0;
	atomic ifTimeSyncStop=START_CYCLE;
	stop_count = STOP_DUTY_CYCLE;
//	tree_parent = call Routing.getRoutingTree();
}

inline void* getFooter(message_t* msg)
{
    return msg->data + payloadLength(msg);
}
uint8_t payloadLength(message_t* msg)
{
    return (call CC2420PacketBody.getHeader(msg))->length - CC2420_SIZE - sizeof(timesync_radio_t);
}

command void TimeSync.start()
{
	atomic ifTimeSyncStop=START_CYCLE;
	initializeParameters();

	call Timer2.startPeriodic(SYNC_PERIOD); 	// Start timer periodically
	if((call RadioControl.start()) != SUCCESS){
		if(TOS_NODE_ID == 1){
			Tw=TWK;				// Set the wakeup interval
			globalTime = (call Timer0.getNow()) - (timeSyncOffset);	// Find global time.
			dutyCycleOffset = globalTime%(Tw + Tsl);	// Calculate the offset of duty cycle with respect to global time.
			if(dutyCycleOffset < Tw){
				wakeSyncOffset = (Tw - dutyCycleOffset);	// Set the next wakeup appropriately
			}
			else{
				wakeSyncOffset = (Tw + Tw + Tsl - dutyCycleOffset);
			}
			call Timer1.start(wakeSyncOffset);	// This timer will toggle radio
		}
		signal TimeSync.startDone(SUCCESS);
	}
	else{
		signal TimeSync.startDone(SUCCESS); // Signal startDone event.
	}
}

command void TimeSync.stop()
{
	atomic ifTimeSyncStop=STOP_CYCLE; 	// Set the flag indicating not to duty cycle.
	call Leds.set(4);

	call RadioControl.start(); 	//Start the radio
	signal TimeSync.stopDone(SUCCESS);	// Signal stopDone event.
}


/*
	Empty AMControl Events
*/
event void AMControl.startDone(error_t err){
}

event void AMControl.stopDone(error_t err) {
}

event void RadioControl.startDone(error_t err)
{
	if(TOS_NODE_ID == 1){
		Tw=TWK;	// Set the dutycycle for head node. Same as explained above.
		globalTime = (call Timer0.getNow()) - (timeSyncOffset);
		dutyCycleOffset = globalTime%(Tw + Tsl);
		if(dutyCycleOffset < Tw){
			wakeSyncOffset = (Tw - dutyCycleOffset);
		}
		else{
			wakeSyncOffset = (Tw + Tw + Tsl - dutyCycleOffset);
		}
		call Timer1.start(wakeSyncOffset);
	}
}

event void RadioControl.stopDone(error_t err) {
	call Timer1.start(Tsl);	// Set sleep interval.
	Tsl=TSL;
}


async event void Timer0.fired(){}

/*
	This timer toggles the radio if it is duty cycling.
*/
async event void Timer1.fired() {
	if(ifTimeSyncStop == START_CYCLE){
		// Toggle radio if it is dutycycling
		if(flag == 1){
			call RadioControl.stop();
			call Leds.set(1);
		}
		else{
			call RadioControl.start();
			call Leds.set(4);
		}
		flag=!flag;
	}
}


/*
	This timer will send timy synchronization packets. It will send the packets whether to duty cycle or not.
*/

event void Timer2.fired() {

	if(locked){
		return;
	}
	else {
		timesync_msg_t* rcm = (timesync_msg_t*)call TimeSyncAMSendI.getPayload(&packet, sizeof(timesync_msg_t));

		if (rcm == NULL) {
			return;
		}

		// Set the fields of the packet.
		rcm->type = ifTimeSyncStop;
		rcm->src=TOS_NODE_ID;
		rcm->offset=timeSyncOffset;
		if(ifTimeSyncStop == START_CYCLE || ( ifTimeSyncStop == STOP_CYCLE && stop_count > 0)){
			if(synchronized == 1 || TOS_NODE_ID == 1){	// Check whether the node is in sync with parent or not
				if (call TimeSyncAMSendI.send(AM_BROADCAST_ADDR, &packet, sizeof(timesync_msg_t),rcm -> sendtime ) == SUCCESS) {
					locked = TRUE;
					if(ifTimeSyncStop == STOP_CYCLE){
						stop_count--; // Decrement transmission count
					}
				}
			}
		}
	}
}

/*

	CaptureSFD.captured() will cause as interrupt when the first byte leaves the radio.
	It adds the timestamp in last 4 bytes of the packet.
*/

async event void CaptureSFD.captured(uint16_t time) {
	uint32_t timer32=(call Timer0.getNow());
	call CSN.clr();
	call TXFIFO_RAM.write(call PacketTimeSyncOffset.get(&packet),(uint8_t*)timer32,sizeof(uint32_t));
	call CSN.set();
}

/*
	Event receive will receive the packets and it will act according to the packets received.
*/

event message_t* Receive.receive(message_t* bufPtr,void* payload, uint8_t len) {

	uint32_t send_time;
	uint32_t cur_recvtime;
	if (len != sizeof(timesync_msg_t)) {return bufPtr;}
	else
	{
		timesync_msg_t* rcm = (timesync_msg_t*)payload;
		if(rcm->type == START_CYCLE){ // The packet indicates to start duty cycle.

			timesync_radio_t* timesync = getFooter(bufPtr);
			atomic ifTimeSyncStop = START_CYCLE;
			cur_recvtime = call PacketTimeStamp32khz.timestamp(bufPtr);
			send_time=(uint32_t)(*timesync) ;
			send_time=(~((~send_time) - rcm->offset)); // Get the offset from the packet.
		//	if(rcm->src != (call Routing.getParent(TOS_NODE_ID))){
			if(rcm->src != tree_parent[(TOS_NODE_ID - 1)]){
				return bufPtr;
			}
			// It calculates the offset of its clock and global clock of Head node.
			// Here the byte order changes so we need to negate the value received in the packet.

			timeSyncOffset = (cur_recvtime-(~send_time));

			Tw=TWK;
			globalTime = (call Timer0.getNow()) - (timeSyncOffset); // Calculate global time
			dutyCycleOffset = globalTime%(Tw + Tsl); // Calculate duty cycle offset with respect to global clock
			if(dutyCycleOffset < Tw){		// Set the wake sync offset based on the duty cycle offset
				wakeSyncOffset = (Tw - dutyCycleOffset);
				call Timer1.start(wakeSyncOffset);
				synchronized=1;
			}
			else{
				Tsl= Tw + Tsl - dutyCycleOffset;
				atomic flag=!flag;
				call Timer1.start(Tsl);
				call RadioControl.stop();
				synchronized=1;
			}
		}
		else if(rcm->type == STOP_CYCLE){ // It indicates to stop duty cycle.
		//	if(rcm->src != (call Routing.getParent(TOS_NODE_ID))){
			if(rcm->src != tree_parent[(TOS_NODE_ID - 1)]){
				return bufPtr;
			}
			atomic ifTimeSyncStop=STOP_CYCLE;
			if(stop_count == 0){ // Send command to children nodes to stop duty cycle.
				stop_count = STOP_DUTY_CYCLE;
			}
			call RadioControl.start(); // Start the radio
			call Leds.set(4);
		}
		return bufPtr;
    	}
}


/*
	Events signalling end of events
*/
event void TimeSyncAMSendI.sendDone(message_t* bufPtr, error_t error) {
	if (&packet == bufPtr) {
		locked = FALSE;
	}
}

event void Routing.routeDone(error_t error){
}

}
