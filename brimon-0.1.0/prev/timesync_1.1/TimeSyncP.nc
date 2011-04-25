/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya, Jeet Patani
 * Copyright (C) 2011 Abhinav Maurya, Jeet Patani
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
		interface Receive[am_id_t id];
		interface SplitControl as AMControl;
		interface Packet;
		interface PacketTimeStamp <T32khz, uint32_t> as PacketTimeStamp32khz;
		interface GpioCapture as CaptureSFD;
		interface GeneralIO as CSN;
		interface CC2420Ram as TXFIFO_RAM;
		interface PacketTimeSyncOffset;
		interface CC2420PacketBody;
		interface SplitControl as RadioControl;
	}
}

implementation {
	message_t packet;
	bool locked;
	uint16_t counter=0;
	uint32_t Tcc, Tdel, Tw=TWK, Tsl=TSL;
	uint8_t flag=1;
	uint8_t parCommandType = 0;
	uint32_t curSendTime = 0;
	uint32_t timeSyncOffset = 0;
	uint32_t nextWakeTime = 0;
	uint32_t globalTime = 0;
	uint32_t wakeSyncOffset = 0;
	int tree_parent[NUM_NODES]={0,1,2,3,4,5};
	uint8_t synchronized = 0;
	uint32_t dutyCycleOffset=0;
	uint8_t ifTimeSyncStop=0;

	inline void* getFooter(message_t* msg);
	uint8_t payloadLength(message_t* msg);

	void initializeParameters() {
		locked=0;
		counter=0;
		Tw=TWK;
		Tsl=TSL;
		flag=1;
		parCommandType = 0;
		curSendTime = 0;
		timeSyncOffset = 0;
		nextWakeTime = 0;
		globalTime = 0;
		wakeSyncOffset = 0;
		synchronized = 0;
		dutyCycleOffset=0;
		ifTimeSyncStop=0;
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
		atomic ifTimeSyncStop=0;
		initializeParameters();
		if((call AMControl.start()) != SUCCESS){
			call Timer2.startPeriodic(SYNC_PERIOD);
			call Leds.set(4);
		}
		signal TimeSync.startDone(SUCCESS);
	}

	command void TimeSync.stop()
	{
		atomic ifTimeSyncStop=1;
		call AMControl.start();
		call Timer0.stop();
		call Timer1.stop();
		call Timer2.stop();
		signal TimeSync.stopDone(SUCCESS);
	}

	event void AMControl.startDone(error_t err)
	{
		if (err == SUCCESS)
		{
			call Timer2.startPeriodic(SYNC_PERIOD);
			call Leds.set(1);
		}
    	else
    	{
      		call AMControl.start();
    	}
  	}

  	event void AMControl.stopDone(error_t err) {}

	event void RadioControl.startDone(error_t err)
	{
		if(TOS_NODE_ID == 1){
		Tw=TWK;
		globalTime = (call Timer0.getNow()) - (timeSyncOffset);
		dutyCycleOffset = globalTime%(Tw + Tsl);
		if(dutyCycleOffset < Tw){
			wakeSyncOffset = (Tw - dutyCycleOffset);
//			printf("Awake : ");
		}
		else{
			wakeSyncOffset = (Tw + Tw + Tsl - dutyCycleOffset);
//			printf("Sleep : ");
		}
//		printf("\n%d --- %lu --- %lu \n",TOS_NODE_ID,globalTime,wakeSyncOffset);
//		printfflush();
		call Timer1.start(wakeSyncOffset);
		call Leds.set(1);
		}
	}

	event void RadioControl.stopDone(error_t err) {
		//printf("Next Wake Time: %hu ... % hu ... %hu ... %lu\n",nextWakeTime, Tsl, Tw, timeSyncOffset);
		//printfflush();
		call Leds.set(0);
		call Timer1.start(Tsl);
		Tsl=TSL;
	}


	async event void Timer0.fired(){}

	async event void Timer1.fired() {
		if(ifTimeSyncStop == 0){
		if(flag == 1)
			call RadioControl.stop();
		else
			call RadioControl.start();
		flag=!flag;
		}
	}

	event void Timer2.fired() {

		counter++;
		//counter=counter%4;

		if(locked){
			return;
		}
		else {
      			timesync_msg_t* rcm = (timesync_msg_t*)call TimeSyncAMSendI.getPayload(&packet, sizeof(timesync_msg_t));

      			if (rcm == NULL) {
				return;
      			}

			rcm->count = counter;
			rcm->src=TOS_NODE_ID;
			rcm->offset=timeSyncOffset;
			if(synchronized == 1 || TOS_NODE_ID == 1){
			if (call TimeSyncAMSendI.send(AM_BROADCAST_ADDR, &packet, sizeof(timesync_msg_t),rcm -> sendtime ) == SUCCESS) {
				locked = TRUE;
			//	printf("-");
			//	printfflush();
    			}
			}
    		}
	}

	async event void CaptureSFD.captured(uint16_t time) {
		uint32_t timer32=(call Timer0.getNow());
	//	printf("Timestamping: %d -- %lu\n", counter, (timer32-timeSyncOffset));
	//	printfflush();
		call CSN.clr();
      		call TXFIFO_RAM.write(call PacketTimeSyncOffset.get(&packet),(uint8_t*)timer32,sizeof(uint32_t));
      		call CSN.set();
 	 }


	event message_t* Receive.receive[am_id_t id](message_t* bufPtr,void* payload, uint8_t len) {

		uint32_t send_time;
		uint32_t cur_recvtime;
    		if (len != sizeof(timesync_msg_t)) {return bufPtr;}
    		else
    		{

			timesync_msg_t* rcm = (timesync_msg_t*)payload;
			timesync_radio_t* timesync = getFooter(bufPtr);
			cur_recvtime = call PacketTimeStamp32khz.timestamp(bufPtr);
			send_time=(uint32_t)(*timesync) ;
			send_time=(~((~send_time) - rcm->offset));
			printf("%d",rcm->src);
			printfflush();
		//	printf("Received Packet from %d -- Parent %d \n",rcm->src,tree_parent[TOS_NODE_ID - 1]);
		//	printfflush();
			if(rcm->src != tree_parent[TOS_NODE_ID - 1]){
				return bufPtr;
			}
			call Leds.set(rcm->count);
		//	printf("Command : %d\n",rcm->count);
		//	printfflush();
			timeSyncOffset = (cur_recvtime-(~send_time));

		//	printf("Count : %d -- Send Time : %lu -- Receive Time : %lu -- Diff : %lu -- %lu -- SyncOffset : %lu\n",rcm->count,~send_time,cur_recvtime,(cur_recvtime-(~send_time)),((~send_time)-cur_recvtime),timeSyncOffset);
		//	printfflush();
		//	if(synchronized == 0){
				Tw=TWK;
				globalTime = (call Timer0.getNow()) - (timeSyncOffset);
				dutyCycleOffset = globalTime%(Tw + Tsl);
				if(dutyCycleOffset < Tw){
					wakeSyncOffset = (Tw - dutyCycleOffset);
		//			printf("C : %d -- ST : %lu -- RT : %lu -- Df : %lu -- %lu -- SO : %lu\n",rcm->count,~send_time,cur_recvtime,(cur_recvtime-(~send_time)),((~send_time)-cur_recvtime),timeSyncOffset);
		//			printf("\n FT : %d--%lu--%lu--%lu--%lu\n ", TOS_NODE_ID, globalTime, timeSyncOffset, wakeSyncOffset, dutyCycleOffset);
		//			printfflush();
					call Timer1.start(wakeSyncOffset);
					call Leds.set(1);
					synchronized=1;
				}
				else{
		//		printf("C : %d -- ST : %lu -- RT : %lu -- Df : %lu -- %lu -- SO : %lu\n",rcm->count,~send_time,cur_recvtime,(cur_recvtime-(~send_time)),((~send_time)-cur_recvtime),timeSyncOffset);
		//			printf("\n NS: %d--%lu--%lu--%lu--%lu\n",TOS_NODE_ID,globalTime,timeSyncOffset,wakeSyncOffset,dutyCycleOffset);
		//			printfflush();
					//wakeSyncOffset = (Tw + Tw + Tsl - dutyCycleOffset);
					Tsl= Tw + Tsl - dutyCycleOffset;
					//wakeSyncOffset = 1;
					flag=!flag;
					call Timer1.start(Tsl);
					call RadioControl.stop();
					//call Leds.set(1);
					synchronized=1;

				}

		//	}
	    		return bufPtr;
    		}
  	}

	event void TimeSyncAMSendI.sendDone(message_t* bufPtr, error_t error) {
		if (&packet == bufPtr) {
      			locked = FALSE;
			 //prev_sendtime = call PacketTimeStamp32khz.timestamp(bufPtr);

    		}
  	}
}
