#include "Timer.h"
#include "CC2420.h"
#include "IEEE802154.h"
#include "UserButton.h"
#include "pipeline.h"


module LoggerM {
  provides interface Logger;
  uses {
    interface SplitControl as Control;
    interface SplitControl as RadioControl;
    interface Leds;


    interface Timer<TMilli> as Timer1;    

    interface CC2420Config;

    interface CC2420PacketBody;       
    interface PacketAcknowledgements;

    interface Notify<button_state_t> as UB;
    interface Alarm<T32khz,uint32_t> as Alarm;    

    interface SerialCom;
    
  }
}

implementation {

  /** logcounter is increamented every time an event is added**/
  uint32_t exptal_log_counter = 0;
  uint32_t logging_log_counter = 0;

  uint32_t send_count; // How many log entries have been sent so far?

  bool shouldSendControlMsg=FALSE;


  /* SB:MOTE to PC functionality */
  void print_msg(char *str, ...) {
    #include<stdio.h>
    #include<stdarg.h>
    va_list ap;
    char buf1[120 - 4];
    serial_pkt_type_t type=DATA_PKT;
    va_start(ap, str);
    vsprintf(buf1,str,ap);
    va_end(ap); 
    call SerialCom.sendToPC(buf1,type,0xFF);// 0xFF: no symantics for code
  }

  event void SerialCom.gotFromPC(void* payload, uint8_t length){}

  /** indicates whether to log current event or not**/
  inline bool shouldLog(uint8_t type) {
    //if(type > EXPECTING_DATA && in_transfer_phase) { return FALSE;}
    return (type < SHOULD_LOG_END);
  } // End shouldLog()

  /** flush the PIP performance log **/
  async command void  Logger.clearExptalLog(){
    uint8_t i=0;
    exptal_log_counter=0;
     
    
    for(i=0;i<NUM_STAT_ENTRIES; i++) {
      statExptalLog[i].type = STAT_END;
      statExptalLog[i].value = 0;
      statExptalLog[i].time = 0;
    }
  }
  /** flush the PIP log collected during log collection***/
  async command void  Logger.clearLoggingLog(){
    uint8_t i=0;
    logging_log_counter=0;
    
    
    for(i=0;i<NUM_STAT_ENTRIES; i++) {
      statLoggingLog[i].type = STAT_END;
      statLoggingLog[i].value = 0;
      statLoggingLog[i].time = 0;
    }
  }



  async command void Logger.reset(){
    send_count=0;
  }
 
  error_t Logger_logData_helper(uint8_t type, uint32_t value);
  async command error_t Logger.logData(uint8_t type, uint32_t value) {
    atomic { return Logger_logData_helper(type, value); }
  }
  error_t Logger_logData_helper(uint8_t type, uint32_t value) {
    /** use of ptr can save on replication done for 2 cases**/
    if(! expt_or_log) {
      if(shouldLog(type) && (type < STAT_END)) {
	statLoggingLog[type].type = type;
	statLoggingLog[type].value = value;
	statLoggingLog[type].time = call Alarm.getNow();
	return SUCCESS;
      }      
      if((logging_log_counter < MAX_LOG_ENTRIES) && shouldLog(type)) {
	loggingLogArray[logging_log_counter].type = type;
	loggingLogArray[logging_log_counter].value = value;
	loggingLogArray[logging_log_counter].time = call Alarm.getNow();
	logging_log_counter++;
      }      
      return SUCCESS;	
    }
    if(shouldLog(type) && (type < STAT_END)) {
      statExptalLog[type].type = type;
      statExptalLog[type].value = value;
      statExptalLog[type].time = call Alarm.getNow();
      return SUCCESS;
    }
    if((exptal_log_counter < MAX_LOG_ENTRIES) && shouldLog(type)) {
      exptalLogArray[exptal_log_counter].type = type;
      exptalLogArray[exptal_log_counter].value = value;
      exptalLogArray[exptal_log_counter].time = call Alarm.getNow();
      exptal_log_counter++;
    }
    return(SUCCESS);
  } // End Logger_logData_helper()

  /** this function is used to fill up the entries in pkts (during log
      collection) **/
  async command error_t Logger.fillNextLogEntry(message_t* pktptr, uint32_t entry_no){
    
    RadioLog *data;
    data= (RadioLog *)  pktptr->data;

    data->seqno=entry_no;
    data->isValid=0xFF; // indicating valid
    
   
    if(entry_no < NUM_STAT_ENTRIES) { 
      data->type=statExptalLog[send_count].type;
      data->value=statExptalLog[send_count].value;
      data->time=statExptalLog[send_count].time;    
    } else if(entry_no < NUM_STAT_ENTRIES + exptal_log_counter) {
      data->type=exptalLogArray[send_count-NUM_STAT_ENTRIES].type;
      data->value=exptalLogArray[send_count-NUM_STAT_ENTRIES].value;
      data->time=exptalLogArray[send_count-NUM_STAT_ENTRIES].time;    
    } else {
      data->isValid=0x00; // indicating pkt contains garbage
    }
    send_count++;
    /** during log collection it has been observed that the log
	entries recvd by the sink after 1st EOF-SNACK phase are
	garbage. Specially when number of packets are increased to
	10000. I tried to debug this by logging the packets
	sent. still no success **/
    if(data->isValid==0xFF){
      call Logger.logData(DEBUG,__LINE__);
      
      call Logger.logData(DEBUG,data->seqno);
      if(entry_no < NUM_STAT_ENTRIES) { 
	call Logger.logData(DEBUG,statExptalLog[send_count].type);
	call Logger.logData(DEBUG,statExptalLog[send_count].value);
	call Logger.logData(DEBUG,statExptalLog[send_count].time);
      }else {
	call Logger.logData(DEBUG,exptalLogArray[send_count-NUM_STAT_ENTRIES].type);
	call Logger.logData(DEBUG,exptalLogArray[send_count-NUM_STAT_ENTRIES].value);
	call Logger.logData(DEBUG,exptalLogArray[send_count-NUM_STAT_ENTRIES].time);

      }
    }

    return SUCCESS;
  }

  event void RadioControl.startDone(error_t error) {    
  } // End RadioControl.startDone();

  /** following is used to dump log over serial. If this function is
      called from tranport module then we need to send the control
      packet to PC to indicate that operation is complete. Otherwise
      no need**/
  async command error_t Logger.dumpLog(bool cald_from_trans_module) {
    send_count=0;
    shouldSendControlMsg=cald_from_trans_module;
    call Timer1.startOneShot(50);
    return SUCCESS;
  } // End Logger.dumpLog()

  void Timer1_fired_helper();
  event void Timer1.fired() {
    atomic { Timer1_fired_helper(); }
  }
  void Timer1_fired_helper() {
    Log toPrint;
    bool local_expt_or_log=expt_or_log;

    // Note now RadioCountToLedsC uses dumplog (for SINK). So be
    // cautious while changing following flag
    local_expt_or_log=TRUE; 

    if(send_count < ((local_expt_or_log ? exptal_log_counter : logging_log_counter) + NUM_STAT_ENTRIES) ) {

      if(send_count < NUM_STAT_ENTRIES) { 
	if(local_expt_or_log){
	  toPrint = statExptalLog[send_count]; 
	} else {
	  toPrint = statLoggingLog[send_count];  
	}
      }else {
	if(!local_expt_or_log){
	  toPrint = loggingLogArray[send_count-NUM_STAT_ENTRIES]; 
	} else {
	  toPrint = exptalLogArray[send_count-NUM_STAT_ENTRIES]; 
	}
      }
      print_msg("%d %lu %d %lu %lu", TOS_NODE_ID, send_count, toPrint.type, toPrint.value, toPrint.time);
      call Leds.set(send_count%8);
      send_count=send_count+1;
      call Timer1.startOneShot(20); // Print with 50ms gap
    } else {
     
      if(shouldSendControlMsg) {
	shouldSendControlMsg=FALSE;
	call SerialCom.sendToPC((char) 0,CONTROL_PKT,DONE_COLLECTING);
	return;
      }
      // retransmit
      send_count = 0;
      /* print_msg("%lu, %lu, %lu", exptal_log_counter, logging_log_counter, send_count);
	 for(send_count=0;send_count<MAX_NEIGHBOURS;send_count++){
	 print_msg("%d %d %d",  ngbrInfoTable.ngbrId[send_count],ngbrInfoTable.avgRSSI[send_count],ngbrInfoTable.numBeaconsReceived[send_count]);
	 }
	 send_count=0;*/
      call Timer1.startOneShot(7000);
    }
  } // End Timer1_fired_helper()


  event void RadioControl.stopDone(error_t error) {		
  } // End RadioControl.stopDone()

  event void Control.startDone(error_t err) {
  }

  event void Control.stopDone(error_t err) {}
 
  event void CC2420Config.syncDone(error_t error) {}


  void UB_notify_helper(button_state_t val); // need to do UB.enable() in order to work 
  event void UB.notify(button_state_t val) {
    atomic { UB_notify_helper(val); }
  }
  void UB_notify_helper(button_state_t val) {
    if(val) {
      send_count=0;
      call Logger.dumpLog(FALSE); // Send over serial 
    }		
  } // End UB_notify_helper()

  async command error_t Logger.init() {
    call UB.enable();
    return SUCCESS;
  } // End Logger.init()

  async event void Alarm.fired() {}


} // End implementation
