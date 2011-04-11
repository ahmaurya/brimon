#include "Timer.h"
#include "RadioCountToLeds.h"
#include "UserButton.h" 
#include "Routing.h"


module RadioCountToLedsC {
  uses {
    interface Leds;
    interface Boot;
    interface Alarm<T32khz,uint32_t> as TransportTimer;
    //interface Timer<TMilli> as MilliTimer;
    interface SplitControl as AMControl;
    interface SplitControl as AMSerialControl;
    //interface CC2420Config;
    //interface CC2420Transmit;
    //interface Receive;
    interface Notify<button_state_t> as UB;
    interface Logger;
    interface PIPMac;
    interface Routing;
    interface BusyWait<TMicro,uint16_t> as BusyWait;
    interface SerialCom;
  }
}

implementation {

  /* SB:MOTE to PC functionality (check MotePCSerialCom folder) */
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
  message_t global_packet;
  bool locked;
  /** The next seq num to check for sending (will be sent if not already acked) */
  uint16_t next_seqno_to_check = 0;
  /** Bit vector of acks, or of received seqno.  The SINK uses this
      array for recdSeqnos and the SRC uses it for acked. */
  uint8_t bitvec_acks_or_recdSeqnos[NUM_PACKS>>3];
  uint8_t *acked;
  uint8_t *recdSeqnos;
  pip_transport_state_t pip_transport_state;
 
  /*** PROTOTYPES*******/
  void sendControlMsgToPC(control_msg_codes_t code);
  void processCommand(serial_msg_t *rcm);
  void send_log_entry_to_pc(message_t * p_msg, uint8_t length);

  void send_data_eof_teardown();
  void send_conn_req();
  void send_snack();
  void reset_transport_state();

  /*********RESET the transport State*********/
  void reset_transport_state(){
    uint8_t i;
    pip_transport_state = PIP_TR_S_INIT;
    for(i = 0; i < (NUM_PACKS>>3); i++) { bitvec_acks_or_recdSeqnos[i] = 0; }    
    next_seqno_to_check = 0;
    in_transfer_phase=TRUE;
    locked=FALSE;
  }


  /**************** Boot, AMControl, AMSerialControl events ****************/
  event void Boot.booted() {
    uint8_t i;
    acked = recdSeqnos = bitvec_acks_or_recdSeqnos;
    for(i = 0; i < (NUM_PACKS>>3); i++) { bitvec_acks_or_recdSeqnos[i] = 0; }
    atomic pip_transport_state = PIP_TR_S_INIT;
    call AMControl.start();
    call AMSerialControl.start(); 
  }
  event void AMSerialControl.startDone(error_t err) {
    if(err != SUCCESS) { call AMSerialControl.start(); }
  }
  event void AMSerialControl.stopDone(error_t err) { }
  event void AMControl.startDone(error_t err) {
    if(err != SUCCESS) { call AMControl.start(); }
  }
  event void AMControl.stopDone(error_t err) { }
  /*=============== End Boot, AMControl, AMSerialControl events ===============*/

  /**************** Helper routines to send various kinds of packets ****************/
  /** Send a connection request packet */
  void send_conn_req() {
    if(call PIPMac.sendConnRequest(&global_packet, sizeof(radio_count_msg_t)) == SUCCESS) {
      locked = TRUE;
    }
  } // End send_conn_req()

  /* Send the next data packet, or EOF, or teardown */
  void send_data_eof_teardown() {
    radio_count_msg_t *rcm = (radio_count_msg_t*)(global_packet.data);
    uint16_t to_send;
    if(! im_source) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(locked) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }

    if(next_seqno_to_check == NUM_PACKS) {
      if(EARLY_TEARDOWN) {
	rcm->seqno = 0; // dummy
	if(call PIPMac.sendTeardown(&global_packet, sizeof(radio_count_msg_t)) == SUCCESS) {
	  locked = TRUE;
	}
	return;
      }

      // We are through with sending this set of packets, send an EOF now
      call Logger.logData(SENDING_EOF, pip_transport_state); 
      rcm->seqno = 0; // dummy
      if(call PIPMac.sendEOF(&global_packet, sizeof(radio_count_msg_t)) == SUCCESS) {
	locked = TRUE;
      }
      return;
    }
    
    /* Calculate the next seqno to be sent */
    for(to_send = next_seqno_to_check; to_send < NUM_PACKS; to_send++) {      
      if( !((acked[to_send/8] >> (to_send%8)) & 0x01) ) { break; }
    } // End for(to_send)

    // There are three cases now...
    if(to_send < NUM_PACKS) {
      // We have found an unacked packet to send
      error_t  checkSend;
      rcm->seqno = to_send;
      next_seqno_to_check = to_send+1;
      
      if(expt_or_log){
	checkSend=call PIPMac.sendData(&global_packet, sizeof(radio_count_msg_t));
      }else{
	call Logger.fillNextLogEntry(&global_packet, (uint32_t)to_send);
	checkSend=call PIPMac.sendData(&global_packet, sizeof(RadioLog));
      }
      if(checkSend==SUCCESS){
	call Leds.set(to_send & 0x07);
	if( ! in_transfer_phase){
	  call Logger.logData(SENDING_DATA, rcm->seqno);
	}

	locked = TRUE;
      }
    } else if(next_seqno_to_check == 0) {
      // We started looking from 0, but found no unacked pkt ==> all acked!
      call Logger.logData(SENDING_TEARDOWN, pip_transport_state);
      rcm->seqno = 0; // dummy
      if(call PIPMac.sendTeardown(&global_packet, sizeof(radio_count_msg_t)) == SUCCESS) {
	locked = TRUE;
      }
    } else {
      // Done with sending this set of pkts; Need to send an EOF
      call Logger.logData(SENDING_EOF, pip_transport_state); 
      rcm->seqno = 0; // dummy
      if(call PIPMac.sendEOF(&global_packet, sizeof(radio_count_msg_t)) == SUCCESS) {
	locked = TRUE;
      }
    }

  } // End send_data_eof_teardown()
  /** Send a SNACK packet */
  void send_snack() {
    radio_count_msg_t *rcm = (radio_count_msg_t*)(global_packet.data);
    uint16_t first_not_recd_index, num_entries, i;
    if(TOS_NODE_ID != SINK) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(locked) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    // Find the first seq num which has not yet been received
    // Actually, find the byte in which that info is stored
    for(first_not_recd_index = 0; first_not_recd_index < (NUM_PACKS>>3); first_not_recd_index++) {
      if(recdSeqnos[first_not_recd_index] != 0xff) { break; }
    }
    rcm->seqno = first_not_recd_index<<3; // note: could be NUM_PACKS
    num_entries = (NUM_PACKS>>3) - first_not_recd_index;
    // Can't send more SNACK entries that what will fit in a packet
    if(num_entries > RCM_DATA_LENGTH) { num_entries = RCM_DATA_LENGTH; }
    rcm->num_entries = num_entries;
    for(i = 0; i < num_entries; i++) {
      rcm->data[i] = recdSeqnos[first_not_recd_index+i];
      if(EARLY_TEARDOWN && PIPStats.num_eof_pkts_recd == 2) {
	if(rcm->data[i] != 0xff) {
	  call Logger.logData(DEBUG, i);
	  call Logger.logData(DEBUG, rcm->data[i]);
	}
      }
    }
    call Logger.logData(SENDING_SNACK, num_entries); 

    for(i =0 ;i < NUM_PACKS; i++){
      if( (recdSeqnos[i/8] & (0x01 << (i%8))) != (0x01 << (i%8)) ) {
	call Logger.logData(EXPECTING_DATA,i);
      }
    }
    //if(call PIPMac.sendSNACK(&global_packet, sizeof(rcm->seqno)+num_entries) == SUCCESS) {
    if(call PIPMac.sendSNACK(&global_packet, sizeof(radio_count_msg_t)) == SUCCESS) {
      locked = TRUE;
    }
  } // End send_snack()

  /* Code needs to go into PIPMac
  */

  /*=============== End Helper routines to send various kinds of packets ===============*/

  /**************** Timer events ****************/
  //event void MilliTimer.fired() { }
  async event void TransportTimer.fired() {
    if(pip_transport_state != PIP_TR_S_SNACKWAIT) {
      call Logger.logData(TRANSPORT_DEBUG, __LINE__); return;
    }
    send_data_eof_teardown(); // will send EOF again
    // Note: timer will be restarted in sendEOFDone()
  } // End TransportTimer.fired()
  /*=============== End Timer events ===============*/

  /**************** PIPMac events ****************/
  async event void PIPMac.startDone() {
    uint8_t i;
    for(i = 0; i < (NUM_PACKS>>3); i++) { bitvec_acks_or_recdSeqnos[i] = 0; }
    
    call Routing.init();    
    if(TOS_NODE_ID != SINK) { return; } // Nothing to do but wait
    //call Routing.startRouting();
    
  } // End PIPMac.startDone()

  async event void PIPMac.sendConnRequestDone(message_t *bufPtr, error_t status) {

    if(TOS_NODE_ID != SINK) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(&global_packet != bufPtr) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    locked = FALSE;
  } // End PIPMac.sendConnRequestDone()

  void PIPMac_sendDataDone_helper(message_t* bufPtr, error_t error);
  async event void PIPMac.sendDataDone(message_t* bufPtr, error_t error) {
    atomic { PIPMac_sendDataDone_helper(bufPtr, error); }
  }
  
  async event void PIPMac.sendHelloMsgDone(message_t *bufPtr, error_t status){}
  async event void PIPMac.incomingHelloMsg(message_t *msg, uint8_t length){}
  async event void PIPMac.sendLSQueryMsgDone(message_t *bufPtr, error_t status){}
  async event void PIPMac.incomingLSQueryMsg(message_t *msg, uint8_t length){}
  async event void PIPMac.sendLSResponseMsgDone(message_t *bufPtr, error_t status){}
  async event void PIPMac.incomingLSResponseMsg(message_t *msg, uint8_t length){}
  async event void PIPMac.sendRtgResetMsgDone(message_t *bufPtr, error_t status){}
  async event void PIPMac.incomingRtgResetMsg(message_t *msg, uint8_t length){}




  void PIPMac_sendDataDone_helper(message_t* bufPtr, error_t error) {
    if(! im_source) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(&global_packet != bufPtr) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    locked = FALSE;
    send_data_eof_teardown();
  } // End PIPMac_sendDone_helper()

  async event void PIPMac.sendEOFDone(message_t *bufPtr, error_t status) {
    uint16_t num_hops;
    uint16_t snack_timeout;
    if(! im_source) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(&global_packet != bufPtr) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    locked = FALSE;
    /* Change state and wait for SNACK */
    pip_transport_state = PIP_TR_S_SNACKWAIT;
    num_hops = MAX_HOPS;
    // Latency is 3/2 per hop; 2 FRAME_PERIOD 's leeway; and overall
    // multiplication with 2 to account for EOF and then SNACK
    snack_timeout = 2 * ((FRAME_PERIOD/2)*3*num_hops + 2*FRAME_PERIOD);
    // Add another factor of 2 leeway for the timeout
    snack_timeout *= 2;
    call Logger.logData(TRANSPORT_DEBUG, __LINE__);
    call TransportTimer.start(snack_timeout);
  } // End PIPMac.sendEOFDone()

  async event void PIPMac.sendSNACKDone(message_t *bufPtr, error_t status) {
    if(TOS_NODE_ID != SINK) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(&global_packet != bufPtr) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    locked = FALSE;
  } // End PIPMac.sendSNACKDone()

  async event void PIPMac.sendTeardownDone(message_t *bufPtr, error_t status) {
    if(! im_source) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(&global_packet != bufPtr) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    locked = FALSE;
    pip_transport_state = PIP_TR_S_INIT; // Done with connection
  } // End PIPMac.sendTeardownDone()

  async event void PIPMac.incomingConnRequest(message_t *p_msg, uint8_t length) {
    if(! im_source) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(pip_transport_state != PIP_TR_S_INIT) {
      call Logger.logData(TRANSPORT_DEBUG, __LINE__); return;
    }
    pip_transport_state = PIP_TR_S_DATASEND;
    next_seqno_to_check = 0;
    send_data_eof_teardown();
  } // End PIPMac.incomingConnRequest()

  async event void PIPMac.incomingData(message_t *p_msg, uint8_t length) {
    radio_count_msg_t *rcm ;    
    if(TOS_NODE_ID != SINK) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    /** If this is lo collection send the entry to pc**/
    if(!expt_or_log){ send_log_entry_to_pc(p_msg, length); return;}
    rcm =(radio_count_msg_t*)(p_msg->data);
    if(rcm->seqno >= NUM_PACKS) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    // Note down seqno which has been recd
    if( ! in_transfer_phase){
      call Logger.logData(GOT_DATA, rcm->seqno);
    }
    recdSeqnos[rcm->seqno/8] |= (0x01 << (rcm->seqno%8));
  } // End PIPMac.incomingData()

  async event void PIPMac.incomingEOF(message_t *p_msg, uint8_t length) {
    if(TOS_NODE_ID != SINK) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    call Logger.logData(RECD_EOF, pip_transport_state);
    call Logger.logData(NUM_PKTS_TILL_NOW,PIPStats.num_pkts_recd);
    atomic in_transfer_phase=FALSE;   
    PIPStats.num_eof_pkts_recd++;
    send_snack(); // Send an appropriate SNACK message
  } // End PIPMac.incomingEOF()

  async event void PIPMac.incomingSNACK(message_t *p_msg, uint8_t length) {
    uint16_t start_seqno;
    //uint8_t num_entries = length-sizeof(start_seqno);
    uint8_t num_entries;
    uint8_t i;
    radio_count_msg_t *rcm = (radio_count_msg_t*)(p_msg->data);
    if(! im_source) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(pip_transport_state != PIP_TR_S_SNACKWAIT) {
      call Logger.logData(TRANSPORT_DEBUG, __LINE__); return;
    }
    
    atomic in_transfer_phase=FALSE;

    PIPStats.num_snack_pkts_recd++;
    call TransportTimer.stop(); // Need to stop the timer which is running
    start_seqno = rcm->seqno;
    num_entries = rcm->num_entries;
    call Logger.logData(RECD_SNACK, num_entries);
    if((start_seqno%8) != 0) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    if(start_seqno+num_entries*8 > NUM_PACKS) {
      call Logger.logData(TRANSPORT_DEBUG, __LINE__); return;
    }
    if(num_entries > RCM_DATA_LENGTH) {
      call Logger.logData(TRANSPORT_DEBUG, __LINE__); return;
    }
    /** SB : MARK the packets as acked. Following for loop was missing**/
    for(i = 0; i < (start_seqno>>3); i++){
      acked[i] = 0xff;
    }
    for(i = 0; i < num_entries; i++) {
      acked[(start_seqno>>3)+i] |= rcm->data[i];
      if(EARLY_TEARDOWN && PIPStats.num_eof_pkts_sent == 2) {
	if(rcm->data[i] != 0xff) {
	  call Logger.logData(DEBUG, i);
	  call Logger.logData(DEBUG, rcm->data[i]);
	}
      }
    } // End for(i)
    if(EARLY_TEARDOWN) {
      call Logger.logData(EARLY_TEARDOWN_WARN, num_entries);
      for(i = 0; i < (NUM_PACKS>>3); i++) { acked[i] = 0xff; }
    }
    pip_transport_state = PIP_TR_S_DATASEND;
    next_seqno_to_check = 0;
    send_data_eof_teardown();
  } // End PIPMac.incomingSNACK()

  async event void PIPMac.incomingTeardown(message_t *p_msg, uint8_t length) {
    if(TOS_NODE_ID != SINK) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
   
    pip_transport_state = PIP_TR_S_INIT; // Done!
    call Logger.logData(RECD_TEARDOWN, pip_transport_state);
    call PIPMac.reset(PIP_S_DEFAULT);
    if(! expt_or_log) {
      sendControlMsgToPC(DONE_COLLECTING);
    }
     
  } // End PIPMac.incomingTeardown()


  async event void PIPMac.resetDone(){

    reset_transport_state();
  }
  /*=============== End PIPMac events ===============*/
  /******ROUTING EVENTS*******/
  /*** After routing reset PIPMac***/
  async event void Routing.routingDone(){
    call PIPMac.reset(PIP_S_DEFAULT);
    /** send msg to PC over control channel**/
    sendControlMsgToPC(DONE_ROUTING);
  }

  /**************** Other misc (unused) events ****************/
  event void UB.notify(button_state_t val) { }

 

  /*** PC-MOTE Communication **/
  /*** following func realizes control channel between mote & Pc**/
  void sendControlMsgToPC(control_msg_codes_t code){
    call SerialCom.sendToPC((char) 0,CONTROL_PKT,code);
  }
  /** do operation as issued  by PCTool**/
  void processCommand(serial_msg_t *rcm){
    SRC=rcm->data[0];  // First byte always indicates SRC ID for any pkt
    if(rcm->code==COLLECT_LOG){
      /** of current operation is collection then set the flag to
	  indicate which logging array to use**/
      atomic  expt_or_log=FALSE;
      if(SRC!=TOS_NODE_ID) {
	send_conn_req(); // This is log query pkt
      } else {
	call Logger.dumpLog(TRUE);
      }
 	          
    } else if (rcm->code==START_TRANSFER){
      expt_or_log=TRUE;
      call Logger.clearExptalLog();
      send_conn_req(); //this is connection req packet
      sendControlMsgToPC(TRANSFER_STARTED);
    } else if (rcm->code==START_ROUTING){
      call Logger.clearExptalLog();
      call Routing.startRouting();
    } else if (rcm->code==GET_ROUTING_TREE){
      uint8_t i;
      print_msg("NODEID\tPARENT\tGOOD_PATH(y/n)");
      for(i=0;i<MAX_NODE_ID; i++){
	if(currTree.nodeid[i]==0xFF){ 
	  break;
	} else {
	  if(currTree.isConnectedThrGoodPath[i]==FALSE){
	    print_msg("%d \t %d \tn",currTree.nodeid[i], currTree.parentOfNode[i]);
	  } else {
	    print_msg("%d \t %d \ty",currTree.nodeid[i], currTree.parentOfNode[i]);
	  }
	} 
      }
      sendControlMsgToPC(SENT_ROUTING_TREE);
    } else if (rcm->code==RESET_RTG_STATE){
      /* broadcst reset routing pkt**/
      call PIPMac.sendRtgResetMsg(&global_packet, sizeof(radio_count_msg_t));
      call Routing.init();
      sendControlMsgToPC(RTG_RESET_DONE);
    } else if (rcm->code==GET_ROUTING_LOG){
      call Logger.dumpLog(TRUE);
    } else if (rcm->code==GET_PATH_OF_SRC){
      ConnPktPayload_t payload;
      uint8_t i;
      call Routing.fillPathAndChannelInfo(SRC,&payload);
      for(i=0;i<payload.totalHopsInPath;i++) {
	print_msg("%d",payload.path[i]);
      }
      sendControlMsgToPC(PATH_SENT); 
    } else{
      sendControlMsgToPC(UNKNOWN_COMMAND);
    }
    
  }

  /** Used during log collection to send the log entries to pc immediately **/
  void  send_log_entry_to_pc(message_t *pktptr, uint8_t length){
    RadioLog *data;
   
    data= (RadioLog *)  pktptr->data;

    if(data->seqno >= NUM_PACKS) { call Logger.logData(TRANSPORT_DEBUG, __LINE__); return; }
    recdSeqnos[data->seqno/8] |= (0x01 << (data->seqno%8));
   
    if( ((uint8_t) data->isValid) ==0xFF) {
      print_msg("%d %lu %d %lu %lu", SRC, (uint32_t)data->seqno,  (uint8_t)data->type,  (uint32_t)data->value,  (uint32_t)data->time);
    }
 
  }
  /** This is function to process commands recvd from the pc**/
  event void SerialCom.gotFromPC(void* payload, uint8_t length){
    serial_msg_t* rcm = (serial_msg_t*)payload;
    call Leds.set(rcm->type);
    
    if (rcm->type!=CONTROL_PKT){      
      print_msg("BAD_PACKET",length);
    }else{
      processCommand(rcm);
    }
  }


} // End implementation
