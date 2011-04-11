# include "Timer.h"
# include <printf.h>
# include "TimeSync.h"
# include "CC2420TimeSyncMessage.h"
module TimeSyncC {
  uses {
	interface Leds;
	interface Boot;
	interface Alarm <T32khz,uint32_t> as Timer0;
	interface TimeSyncAMSend<T32khz,uint32_t> as TimeSyncAMSendI;
	interface TimeSyncPacket<T32khz,uint32_t> ;
	interface Receive[am_id_t id];
	interface SplitControl as AMControl;	
	interface Packet;	
	interface PacketTimeStamp <T32khz, uint32_t> as PacketTimeStamp32khz;
	interface Timer<TMilli> as Timer1;
	interface GpioCapture as CaptureSFD;

	interface GeneralIO as CSN;

	interface CC2420Ram as TXFIFO_RAM;

	interface PacketTimeSyncOffset;

	interface CC2420PacketBody;
	}
}

implementation {

	message_t packet;
	bool locked;
	inline void* getFooter(message_t* msg);
	uint8_t payloadLength(message_t* msg);
	uint16_t counter=0;
	uint32_t prev_sendtime=0;
	uint32_t s[10];
	uint32_t r[10];
	uint32_t d1[10];
	uint32_t d2[10];
	uint32_t c[10];
	index=0;


  inline uint32_t time16to32(uint16_t time, uint32_t recent_time) {
    if ((recent_time&0xFFFF)<time)
      return ((recent_time-0x10000UL)&0xFFFF0000UL)|time;
    else
      return (recent_time&0xFFFF0000UL)|time;
  } // End time16to32

	inline void* getFooter(message_t* msg){
	    return msg->data + payloadLength(msg);
	  }
	  uint8_t payloadLength(message_t* msg)
	  {
	    return (call CC2420PacketBody.getHeader(msg))->length - CC2420_SIZE - sizeof(timesync_radio_t);
	  }

	
	event void Boot.booted() {
		call AMControl.start();
	}


	event void AMControl.startDone(error_t err) {
		if (err == SUCCESS) {
			call Timer1.startPeriodic(100);
		}
    		else {
      			call AMControl.start();
    		}
  	}

	async event void Timer0.fired(){}
	event void Timer1.fired() {
		
		counter++;
		counter=counter%8;
		

		if(locked){
			return;	
		}
		else {
      			timesync_msg_t* rcm = (timesync_msg_t*)call TimeSyncAMSendI.getPayload(&packet, sizeof(timesync_msg_t));

      			if (rcm == NULL) {
				return;
      			}
			
			rcm->count = counter;
			rcm->prev_sendtime=prev_sendtime;			
			if (call TimeSyncAMSendI.send(AM_BROADCAST_ADDR, &packet, sizeof(timesync_msg_t),rcm -> sendtime ) == SUCCESS) {
				locked = TRUE;
    			}
    		}
	
	}

	async event void CaptureSFD.captured(uint16_t time) {
		uint32_t timer32=call Timer0.getNow();
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
    		else {
			uint8_t* sent, *recd;    			

			timesync_msg_t* rcm = (timesync_msg_t*)payload;

			timesync_radio_t* timesync = getFooter(bufPtr);
			cur_recvtime = call PacketTimeStamp32khz.timestamp(bufPtr);
			send_time=(uint32_t)(*timesync) ;

			call Leds.set(rcm->count);



			printf("Count : %d -- Send Time : %lu -- Receive Time : %lu -- Diff : %lu -- %lu\n",rcm->count,~send_time,cur_recvtime,(cur_recvtime-(~send_time)),((~send_time)-cur_recvtime));
			printfflush();
    			return bufPtr;
    		}
  	}

	event void TimeSyncAMSendI.sendDone(message_t* bufPtr, error_t error) {
		if (&packet == bufPtr) {
      			locked = FALSE;
			 prev_sendtime = call PacketTimeStamp32khz.timestamp(bufPtr);
			
    		}
  	}	
	

 

	event void AMControl.stopDone(error_t err) {
  	}
}
