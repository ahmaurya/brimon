#include "TimeSync.h"

module TimeSyncM {
  uses {
	interface Leds;
	interface Boot;
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
	inline void* getFooter(message_t* msg);
	uint8_t payloadLength(message_t* msg);
	uint16_t counter=0;
	uint32_t Tcc, Tdel, Tw=500000L, Tsl=500000L;
	uint8_t flag=1;
	uint8_t parCommandType = 0;
	uint32_t parWakeTime = 0;
	uint32_t curSendTime = 0;
	uint32_t timeSyncOffset = 0;
	uint32_t nextWakeTime = 0;
	uint32_t globalTime = 0;
	uint32_t wakeSyncOffset = 0;
	int tree_parent[NODES]={0,1,1,2,4,5};
	

 	inline uint32_t time16to32(uint16_t time, uint32_t recent_time)
 	{
	    if ((recent_time&0xFFFF)<time)
	      return ((recent_time-0x10000UL)&0xFFFF0000UL)|time;
	    else
	      return (recent_time&0xFFFF0000UL)|time;
	}

	inline void* getFooter(message_t* msg)
	{
	    return msg->data + payloadLength(msg);
	}  
	  
	uint8_t payloadLength(message_t* msg)
	{
	    return (call CC2420PacketBody.getHeader(msg))->length - CC2420_SIZE - sizeof(timesync_radio_t);
	}

	event void Boot.booted()
	{
		call AMControl.start();
	}

	event void AMControl.startDone(error_t err)
	{
		if (err == SUCCESS)
		{
			call Timer2.startPeriodic(100);
			//call Leds.set(TOS_NODE_ID);
		}
    	else
    	{
      			call AMControl.start();
    	}
  	}
  	
	event void RadioControl.startDone(error_t err)
	{
		globalTime = (call Timer0.getNow()) - timeSyncOffset;
		wakeSyncOffset = (Tw + Tsl - (globalTime%(Tw + Tsl)) );
		printf("%d --- %lu --- %lu \n",NODE_ID,globalTime,wakeSyncOffset);
		printfflush(); 
		call Timer1.start(wakeSyncOffset);
		call Leds.set(4);
		curSendTime = call Timer0.getNow();
		counter++;
	}

	event void RadioControl.stopDone(error_t err) {
		if(parWakeTime!=0)
			nextWakeTime = (parWakeTime - timeSyncOffset + Tsl + Tw - (call Timer0.getNow()))>>5;		
			//nextWakeTime = ( Tsl + Tw - timeSyncOffset );
		else
			nextWakeTime = Tsl;
		//printf("Next Wake Time: %hu ... % hu ... %hu ... %lu\n",nextWakeTime, Tsl, Tw, timeSyncOffset);
		//printfflush();
		call Timer1.start(Tsl);
		call Leds.set(1);
	}
  	

	async event void Timer0.fired(){}
	
	async event void Timer1.fired() {
		if(flag == 1)
			call RadioControl.stop();
		else
			call RadioControl.start();
		
		flag=!flag;
	}
	
	event void Timer2.fired() {
		
		
		counter=counter%4;
		
		if(locked){
			return;	
		}
		else {
      			timesync_msg_t* rcm = (timesync_msg_t*)call TimeSyncAMSendI.getPayload(&packet, sizeof(timesync_msg_t));

      			if (rcm == NULL) {
				return;
      			}
			
			rcm->count = counter;
			rcm->src=NODE_ID;
			if (call TimeSyncAMSendI.send(AM_BROADCAST_ADDR, &packet, sizeof(timesync_msg_t),rcm -> sendtime ) == SUCCESS) {
				locked = TRUE;
    			}
    		}
	}

	async event void CaptureSFD.captured(uint16_t time) {
		uint32_t timer32=(call Timer0.getNow()) - timeSyncOffset;
		nx_uint8_t *taddr;
    		timesync_radio_t *timesync;

		call CSN.clr();
      		call TXFIFO_RAM.write(call PacketTimeSyncOffset.get(&packet),(uint8_t*)timer32,sizeof(uint32_t));
      		call CSN.set();  		
 	 }


	event message_t* Receive.receive[am_id_t id](message_t* bufPtr,void* payload, uint8_t len) {

		uint32_t timerecv;
		uint32_t send_time;
		uint32_t cur_recvtime;	
    		if (len != sizeof(timesync_msg_t)) {return bufPtr;}
    		else
    		{
			
			timesync_msg_t* rcm = (timesync_msg_t*)payload;
			timesync_radio_t* timesync = getFooter(bufPtr);
			cur_recvtime = call PacketTimeStamp32khz.timestamp(bufPtr);
			send_time=(uint32_t)(*timesync) ;
			printf("Received Packet from %d -- Parent %d \n",rcm->src,tree_parent[NODE_ID - 1]);
			printfflush();
			if(rcm->src != tree_parent[NODE_ID - 1]){
				return bufPtr;
			}
			call Leds.set(rcm->count);
			printf("Command : %d\n",rcm->count);
			printfflush();
			//parWakeTime = rcm->sendtime;
			timeSyncOffset = (cur_recvtime-(~send_time));

		//	printf("Count : %d -- Send Time : %lu -- Receive Time : %lu -- Diff : %lu -- %lu -- SyncOffset : %lu\n",rcm->count,~send_time,cur_recvtime,(cur_recvtime-(~send_time)),((~send_time)-cur_recvtime),timeSyncOffset);
		//	printfflush();
    		return bufPtr;
    		}
  	}

	event void TimeSyncAMSendI.sendDone(message_t* bufPtr, error_t error) {
		if (&packet == bufPtr) {
      			locked = FALSE;
			 //prev_sendtime = call PacketTimeStamp32khz.timestamp(bufPtr);
			
    		}
  	}	

	event void AMControl.stopDone(error_t err) {
  	}
}
