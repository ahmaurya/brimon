# include <printf.h>
# include "Timer.h"
# include "CC2420TimeSyncMessage.h"

# include "BriMon.h"

module TimeSyncC {
  uses {
	interface Leds;
	interface Boot;
	interface Alarm <T32khz,uint32_t> as Timer0;
	interface Timer<TMilli> as Timer1;
	interface Alarm <T32khz,uint32_t> as Timer2;
	interface TimeSyncAMSend<T32khz,uint32_t> as TimeSyncAMSendI;
	interface TimeSyncPacket<T32khz,uint32_t> ;
	interface Receive[am_id_t id];
	interface SplitControl as AMControl;	
	interface Packet;
	interface PacketTimeStamp<T32khz, uint32_t> as PacketTimeStamp32khz;
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
	uint32_t counter=0;
	uint32_t Tcc, Tdel, Tw=50000L, Tsl=50000L;
	uint8_t flag=1;
	uint8_t parCommandType = 0;
	uint32_t parWakeTime = 0;
	uint32_t curSendTime = 0;
	uint32_t timeSyncOffset = 0;
	uint32_t nextWakeTime = 0;
	//uint8_t a = 129, *aptr = &a;
	//uint16_t b = 129+(129<<8), *bptr = &b;
	//uint32_t c = 129L+(129L<<8)+(129L<<16)+(129L<<24), *cptr = &c;
	//brimon_msg_t* rcm;
		
	inline void* getFooter(message_t* msg)
	{
	    return msg->data + payloadLength(msg);
	}
	
	inline uint32_t time16to32(uint16_t time, uint32_t recent_time) {
	    if ((recent_time&0xFFFF)<time)
	      return ((recent_time-0x10000UL)&0xFFFF0000UL)|time;
	    else
	      return (recent_time&0xFFFF0000UL)|time;
	} // End time16to32
	  
	uint8_t payloadLength(message_t* msg)
	{
	    return (call CC2420PacketBody.getHeader(msg))->length - CC2420_SIZE - sizeof(timesync_radio_t);
	}

	event void Boot.booted() {
		call AMControl.start();
		}
	
	event void RadioControl.startDone(error_t err) {
		call Timer2.start(Tw);
		call Leds.set(7);
		curSendTime = call Timer0.getNow();
		/*printf("------------------\n");
		printf("%x ... %x ... %x\n", a, b, c);
		printf("%hx ... %hx ... %hx\n", a, b, c);
		printf("%lx ... %lx ... %lx\n", a, b, c);
		printf("%lx\n", a);
		printf("%lx\n", b);
		printf("%lx\n", (uint32_t)a);
		printf("%lx\n", (uint32_t)b);
		printf("%lx\n", c);
		printf("%x ... %x ... %x ... %x ... %x ... %x ... %x\n", *aptr, *bptr, *(bptr+1), *cptr, *(cptr+1), *(cptr+2), *(cptr+3));
		printf("%hx ... %hx ... %hx ... %hx ... %hx ... %hx ... %hx\n", *aptr, *bptr, *(bptr+1), *cptr, *(cptr+1), *(cptr+2), *(cptr+3));
		printf("%lx ... %lx ... %lx ... %lx ... %lx ... %lx ... %lx\n", *aptr, *bptr, *(bptr+1), *cptr, *(cptr+1), *(cptr+2), *(cptr+3));
		printf("\n------------------\n");
		printfflush();*/
	}

	event void RadioControl.stopDone(error_t err) {
		if(parWakeTime!=0)
			nextWakeTime = (parWakeTime - timeSyncOffset + Tsl + Tw - (call Timer0.getNow()))>>5;		
			//nextWakeTime = ( Tsl + Tw - timeSyncOffset );
		else
			nextWakeTime = Tsl;
		//printf("Next Wake Time: %hu ... % hu ... %hu ... %lu\n",nextWakeTime, Tsl, Tw, timeSyncOffset);
		//printfflush();
		call Timer2.start(Tsl);
		call Leds.set(1);
	}

	event void AMControl.startDone(error_t err) {
		if (err == SUCCESS) {
			call Timer1.startPeriodic(1000);
			call Timer2.start(Tw);
		}
    	else {
      			call AMControl.start();
    		}
  	}

	async event void Timer0.fired(){}
	
	async event void Timer2.fired() {
		if(flag == 1)
			call RadioControl.stop();
		else
			call RadioControl.start();
		
		flag=!flag;
	}
	
	event void Timer1.fired() {
		bool status = FALSE;
		
		counter++;
		//counter=counter%8;

		if(locked){
			return;	
		}
		
		else {
      		brimon_msg_t* rcm = (brimon_msg_t*)call TimeSyncAMSendI.getPayload(&packet, sizeof(brimon_msg_t));
      		if (rcm == NULL)
				return;
			rcm->type = 0;
			rcm->count = counter;
			rcm->sendtime = call Timer0.getNow();
			
			status = call TimeSyncAMSendI.send(AM_BROADCAST_ADDR, &packet, sizeof(brimon_msg_t),rcm -> sendtime );	
			if (status == SUCCESS) {
				locked = TRUE;
    			}
    		}
	
	}

	async event void CaptureSFD.captured(uint16_t time) {
	
		//uint32_t timer32 = time16to32(time, call Timer0.getNow());
		
		uint32_t timer32=call Timer0.getNow();
		nx_uint8_t *taddr;
    	timesync_radio_t *timesync;

		//	taddr = packet->data + (call PacketTimeSyncOffset.get(packet) - sizeof(cc2420_header_t));
      	//	timesync = (timesync_radio_t*)taddr;
      	//	*timesync  = timer32;
		call CSN.clr();
      	call TXFIFO_RAM.write(call PacketTimeSyncOffset.get(&packet),(uint8_t*)timer32,sizeof(uint32_t));
      	call CSN.set();	
  		
 	 }

	event message_t* Receive.receive[am_id_t id](message_t* bufPtr,void* payload, uint8_t len) {

		uint32_t timerecv;
		uint32_t send_time;
		uint32_t cur_recvtime;
			
    	if (len != sizeof(brimon_msg_t)) {return bufPtr;}
    	else {
    		brimon_msg_t* rcm = (brimon_msg_t*)payload;
    		timesync_radio_t* timesync = getFooter(bufPtr);
			cur_recvtime = call PacketTimeStamp32khz.timestamp(bufPtr);
			send_time=(uint32_t)(*timesync);
			timeSyncOffset = cur_recvtime-(~send_time);
	
			call Leds.set(rcm->count);
			printf("Count : %d -- Send Time : %lu -- Receive Time : %lu -- Diff : %lu -- %lu\n",rcm->count,~send_time,cur_recvtime,(cur_recvtime-(~send_time)),((~send_time)-cur_recvtime));
			printfflush();
    		return bufPtr;
    	
			parWakeTime = rcm->sendtime;
			parCommandType = rcm->type;
			call Leds.set(rcm->count);
			
			
			
			//printf("----********%d : %lx : %lx : %lx : %lx : %lx-------\n\n\n\n",rcm->count, timeSyncOffset, send_time,~send_time,*taddr,*(taddr+1),*(taddr-1));
			//printf("\n\n%lu : %lu : %lu : %lu : %lu\n\n",rcm->count,~send_time,cur_recvtime,(cur_recvtime-(send_time)),timeSyncOffset);
    	}
  	}

	event void TimeSyncAMSendI.sendDone(message_t* bufPtr, error_t error) {
		if (&packet == bufPtr) {
      			locked = FALSE;
    		}
  	}	
	
	event void AMControl.stopDone(error_t err) {}
  	
}
