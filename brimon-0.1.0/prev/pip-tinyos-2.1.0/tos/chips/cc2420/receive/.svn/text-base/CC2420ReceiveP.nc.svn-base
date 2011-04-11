/*
 * Copyright (c) 2005-2006 Arch Rock Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Arch Rock Corporation nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * ARCHED ROCK OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE
 */

/**
 * @author Jonathan Hui <jhui@archrock.com>
 * @author David Moss
 * @author Jung Il Choi
 * @version $Revision: 1.16 $ $Date: 2008/07/25 16:27:52 $
 */

#include "IEEE802154.h"
#include "message.h"
#include "AM.h"




/* PIP */
#include "pipeline.h"

module CC2420ReceiveP @safe() {

  provides interface Init;
  provides interface StdControl;
  provides interface CC2420Receive;
  provides interface Receive;
  provides interface ReceiveIndicator as PacketIndicator;

  /* PIP */
  provides interface PIPReceive;

  uses interface GeneralIO as CSN;
  uses interface GeneralIO as FIFO;
  uses interface GeneralIO as FIFOP;
  uses interface GpioInterrupt as InterruptFIFOP;

  //uses interface Resource as SpiResource; // Included in PIPTransmit
  uses interface CC2420Fifo as RXFIFO;
  uses interface CC2420Strobe as SACK;
  uses interface CC2420Strobe as SFLUSHRX;

  uses interface CC2420Packet;
  uses interface CC2420PacketBody;
  uses interface CC2420Config;
  uses interface PacketTimeStamp<T32khz,uint32_t>;

  uses interface Leds;

  /* PIP */
  uses interface Alarm<T32khz,uint32_t> as Alarm;
  uses interface PacketTimeStamp<T32khz,uint32_t> as PacketTimeStamp32khz;
  uses interface AMPacket; 
  uses interface PacketAcknowledgements;


  /* PIP */
  uses interface PIPTransmit;
  uses interface Logger;
  uses interface BusyWait<TMicro,uint16_t> as BusyWait;


}

implementation {

  enum {
    RXFIFO_SIZE = 128,
    TIMESTAMP_QUEUE_SIZE = 8,
    SACK_HEADER_LENGTH = 7,
  };

  uint32_t m_timestamp_queue[ TIMESTAMP_QUEUE_SIZE ];
  uint8_t m_timestamp_head;
  uint8_t m_timestamp_size;
  /***SB: PIP Variable ****/
  bool shouldProcessRecdPkt=FALSE;
  

  /** The length of the frame we're currently receiving */
  norace uint8_t rxFrameLength;
  norace uint8_t m_bytes_left; // What is this?
  norace message_t* ONE_NOK m_p_rx_buf; // Used for RXFIFO reading

  /**************** Prototypes ****************/
  void reset_state();
  void resetPIPVars();
  void receiveSPI();
  void RXFIFOFlush();
  bool passesAddressCheck(message_t * ONE msg);
  void processRecdPkt(); // Process the pkt recd over radio and then  read from SPI
  void acquireSpiResourceDone();
  void eventTime(message_t* msg);
  inline void* getFooter(message_t* msg);
  uint8_t payloadLength(message_t* msg);
  /*=============== End Prototypes ===============*/

  /* Flush out the RXFIFO */
  void RXFIFOFlush() {
    call CSN.set();
    call CSN.clr();
    call SFLUSHRX.strobe();
    call SFLUSHRX.strobe();
    call CSN.set();
    rxfifo_state = FIFO_EMPTY;
    call Logger.logData(SPI_RX_FLUSH, pip_state);
  } // End RXFIFOFlush()

  /**
   * The first byte of each packet is the length byte.  Read in that single
   * byte, and then read in the rest of the packet.  The CC2420 could contain
   * multiple packets that have been buffered up, so if something goes wrong, 
   * we necessarily want to flush out the FIFO unless we have to.
   */
  void receiveSPI() {
    m_bytes_left = RXFIFO_SIZE;
    call CSN.clr();
    //call RXFIFO.beginRead( (uint8_t*)(call CC2420PacketBody.getHeader( m_p_rx_buf )), 1 );
    // PIP: read the length byte directly into rxFrameLength -Bhaskar.
    call RXFIFO.beginRead(&rxFrameLength, 1);
  } // End receiveSPI()

  /**
   * Reset this component
   */
  void reset_state() {
    rx_m_state = RX_S_STARTED;
    m_bytes_left = RXFIFO_SIZE;
    m_timestamp_head = 0;
    m_timestamp_size = 0;
    //rxfifo_state = FIFO_EMPTY;
  } // End reset_state()

  /*** If recvd packet is corrupt abort the SPI read & switch to rx mode**/
  void abortSPIRead() {

    reset_state();
    //rx_m_state = RX_S_STARTED; // Done in reset_state()
    prev_txrad_half_state.spi_work_done = TRUE;
    call PIPReceive.checkAndSwitchToRx();
  } // End abortSPIRead()

  /**
   * @return TRUE if the given message passes address recognition
   */
  bool passesAddressCheck(message_t *msg) {
    cc2420_header_t *header = call CC2420PacketBody.getHeader( msg );
    if(!(call CC2420Config.isAddressRecognitionEnabled())) {
      return TRUE;
    }
    return (header->dest == call CC2420Config.getShortAddr()
	    || header->dest == AM_BROADCAST_ADDR);
  } // End passesAddressCheck()

  /** Fill in metadata details, pass the packet up the stack.  This
      gets executed only when whole packet is received correctly by
      the node (over radio and then over SPI). */
  // Bhaskar: not a task anymore
  // Bhaskar: was earlier receiveSPIDone()
  void processRecdPkt() {
    uint8_t *data = (uint8_t *) m_p_rx_buf->data;
    cc2420_metadata_t* metadata = (cc2420_metadata_t*)(m_p_rx_buf->metadata);
    cc2420_header_t* header = (cc2420_header_t*)(m_p_rx_buf->header);
    pip_header_t *piphdr = (pip_header_t*)(&(header->pip_hdr));//(pip_header_t*)(m_p_rx_buf->data);
    uint8_t length = header->length;
    uint8_t tmpLen __DEPUTY_UNUSED__ = sizeof(message_t) - (offsetof(message_t, data) - sizeof(cc2420_header_t));
    uint8_t* COUNT(tmpLen) buf = TCAST(uint8_t* COUNT(tmpLen), header);
    uint16_t seqno = 0xffff;

    bool data_direction, is_routing_msg;
    uint8_t expected_src;
    bool should_forward = FALSE;
    pip_pkt_type_t type = piphdr->pip_pkt_type;

    if(prev_txrad_half_state.rxspi_crc_error) {
      /** SB: Found a bug. If CONN_PKT recvd with CRC error=> all
       *  subsequent pkts will not be recvd. This is bcos prev_txrad_half_state is
       *  never  reset.  As no pkt is recvd TX_TIMER is never
       *  started=> prev_txrad_half_state is never reset
       *  (Interdependency) So init the struct here.**/
      if(pip_state== PIP_S_RXCONN_REQ  || pip_state==PIP_S_DEFAULT ) { 
	init_txrad_state(pip_state);
      }

      PIPStats.num_crc_err++;
      // Note: the queue entry into which the pkt was read can be reused
      return;
    }


   
    metadata->crc = buf[ length ] >> 7;
    metadata->lqi = buf[ length ] & 0x7f;
    metadata->rssi = buf[ length - 1 ];

    if(!passesAddressCheck(m_p_rx_buf) ||  length < CC2420_SIZE) {
      call Logger.logData(RXDEBUG, __LINE__);
      return;
    }   


    data_direction =
      (type == PIP_PKT_DATA) || (type == PIP_PKT_EOF) || (type == PIP_PKT_TEARDOWN);
   
    expected_src = data_direction ? pip_prevNode : pip_nextNode;  
								 

    is_routing_msg= (type == RTG_HELLOMSG) ||(type == RTG_LSQUERYMSG)||(type == RTG_LSRESPONSEMSG)||(type == RTG_RESET_STATE);


     
    if(header->src != expected_src && !is_routing_msg && type!=PIP_PKT_CONN_REQ) {
      call Logger.logData(RAD_OVERHEARD, header->src);
      call Logger.logData(DEBUG,__LINE__);
      return;
    }
    

    
    if(data_direction &&  !is_routing_msg) {
      eventTime(m_p_rx_buf); // re-computes node_time_offset
    }

    seqno = *((nx_uint16_t*)data);
    call Logger.logData(SPI_RX_DONE, seqno);  
    is_rxspi_in_progress=FALSE;

    if(is_routing_msg){
      if(type == RTG_HELLOMSG) {
	atomic pip_state=PIP_S_ROUTING;
	call PIPTransmit.signalIncomingHelloMsg(m_p_rx_buf, length-LENGTH_FIELD_ADJ);
      } else if (type == RTG_RESET_STATE){
	atomic pip_state=PIP_S_ROUTING;
	call PIPTransmit.signalIncomingRtgResetMsg(m_p_rx_buf, length-LENGTH_FIELD_ADJ);
      } else if(type== RTG_LSQUERYMSG) {
	if(pip_state!=PIP_S_ROUTING){
	  call Logger.logData(RXDEBUG, pip_state);
	}
	call PIPTransmit.signalIncomingLSQueryMsg(m_p_rx_buf, length-LENGTH_FIELD_ADJ);
      } else if(type==RTG_LSRESPONSEMSG){
	if(pip_state!=PIP_S_ROUTING){
	  call Logger.logData(RXDEBUG, pip_state);
	}
	call PIPTransmit.signalIncomingLSResponseMsg(m_p_rx_buf, length-LENGTH_FIELD_ADJ);
      }
      return; 
    }


    if(type == PIP_PKT_CONN_REQ) {
      ConnPktPayload_t *payloadptr= (ConnPktPayload_t*) data;
      call Logger.logData(RX_CONN_REQ, piphdr->datasrc);
      num_conn_req_recd++;
      
      /** check whether pkt is conn_req or log_query**/
      if(payloadptr->connreq_or_logquery==88) {
	atomic expt_or_log=TRUE;
	call Logger.clearExptalLog();
      } else if(payloadptr->connreq_or_logquery==99) {
	atomic expt_or_log=FALSE;
	call Logger.clearLoggingLog();
      } else {
	call Logger.logData(RXDEBUG, __LINE__);
      }
      pip_nextNodeRxChannel=payloadptr->channelInfo[payloadptr->noHopsToGo+1];
      
      if( (payloadptr->noHopsToGo+1) == payloadptr->totalHopsInPath) {
	pip_nextNode=SINK; 
      }else{
	pip_nextNode=payloadptr->path[payloadptr->noHopsToGo+1]; //should be equal to header->src
      }
      pip_myRxChannel=payloadptr->channelInfo[payloadptr->noHopsToGo];
      
      my_depth=payloadptr->totalHopsInPath-payloadptr->noHopsToGo;

      payloadptr->noHopsToGo--;
      
      // check whether node is source for the transfer
      if(TOS_NODE_ID==piphdr->datasrc) { 
	atomic time_synced = TRUE; // source is always time_synced
	atomic im_source=TRUE; 
      } else {
	pip_prevNode=payloadptr-> path[payloadptr->noHopsToGo];
	pip_prevNodeRxChannel=payloadptr-> channelInfo[payloadptr->noHopsToGo];
	call Logger.logData(DEBUG,__LINE__);
	call Logger.logData(DEBUG,pip_prevNode);
	call Logger.logData(DEBUG,pip_prevNodeRxChannel);
      }
	  
      if(num_conn_req_recd >= CONN_REQ_REPEAT) {
	call PIPTransmit.TxRTimer_stop();
	if(TOS_NODE_ID == piphdr->datasrc) {
	  pip_state = PIP_S_RXRAD_HALF;
	  call PIPTransmit.signalIncomingConnReq(m_p_rx_buf, length-LENGTH_FIELD_ADJ);
	} else {
	  pip_state = PIP_S_TXCONN_REQ;
	  should_forward = TRUE;
	}
	call Logger.logData(PIP_STATE_LOG, pip_state);
      } else {
	pip_state = PIP_S_RXCONN_REQ;
	call Logger.logData(PIP_STATE_LOG, pip_state);
	if(!call PIPTransmit.TxRTimer_isRunning()) {
	  // Should wait for about the time required to transmit
	  // CONN_REQ_REPEAT-1 more connection requests; extra
	  // FRAME_PERIOD for leeway
	  call PIPTransmit.TxRTimer_start(CONN_REQ_REPEAT*FRAME_PERIOD);
	}
      } // End else of if(num_conn_req_recd >= CONN_REQ_REPEAT)
    } // End if(pkt type is PIP_PKT_CONN_REQ)

    PIPStats.num_pkts_recd++;

    if(type == PIP_PKT_DATA) {
      seqno = *((nx_uint16_t*)data);
      should_forward = (TOS_NODE_ID != piphdr->datadest);
      //call Logger.logData(RXDEBUG, seqno);

     
     
      if(PIPStats.num_data_pkts_recd == 0) {
	call Logger.logData(STAT_FIRST_PKT, seqno);
      }
      call Logger.logData(STAT_LAST_PKT, seqno);
      PIPStats.num_data_pkts_recd++;
      call Leds.set(PIPStats.num_data_pkts_recd%8);
    

      if(!should_forward) {	
	call PIPTransmit.signalIncomingData(m_p_rx_buf, length-LENGTH_FIELD_ADJ);
      } // End if(!should_forward)
   
    } // End if(pkt type is PIP_PKT_DATA)

    if(type == PIP_PKT_EOF) {
      should_forward = (TOS_NODE_ID != SINK);
      if(!should_forward) {	
	call PIPTransmit.signalIncomingEOF(m_p_rx_buf, length-LENGTH_FIELD_ADJ);
      } else {
	call Logger.logData(FWD_EOF, q_len);
      }
    } // End if(pkt type is PIP_PKT_EOF)

    if(type == PIP_PKT_SNACK) {
      should_forward = (!im_source);
      if(!should_forward) {	
	call PIPTransmit.signalIncomingSNACK(m_p_rx_buf, length-LENGTH_FIELD_ADJ);
      } else {
	call Logger.logData(FWD_SNACK, q_len);
	in_transfer_phase=FALSE;
      }
    } // End if(pkt type is PIP_PKT_SNACK)

    if(type == PIP_PKT_TEARDOWN) {

      // Note: only the first teardown pkt should be forwarded, others
      // should be suppressed
      should_forward = ((TOS_NODE_ID != SINK) && (num_tear_down_recd == 0));
      num_tear_down_recd++;
      if(TOS_NODE_ID == SINK) {
	call PIPReceive.connectionDone();
	if(num_tear_down_recd==1) {
	  // signal for first pkt only. Anyway after switching we wont
	  // get anymore teardowns
	  call PIPTransmit.setChannel(PIP_DEFAULT_CHANNEL);
	  call PIPTransmit.signalIncomingTeardown(m_p_rx_buf, length-LENGTH_FIELD_ADJ);
	}
	
      }     
      if(should_forward) {
	call Logger.logData(FWD_TEARDOWN, q_len);
      }
    } // End if(pkt type is PIP_PKT_TEARDOWN)

    if(should_forward) {
      // Pkt needs to be forwarded, add the received packet to the queue
      rear = (rear+1)%CIRC_QUEUE_ARRAY_SIZE; q_len++;
      // Re-adjust addresses in the packet
      header->dest = header->destpan =
	data_direction ? pip_nextNode : pip_prevNode;
      header->src = TOS_NODE_ID;
      header->fcf |= 1 << IEEE154_FCF_ACK_REQ;
      if(type == PIP_PKT_CONN_REQ) { TX_ATTEMPTS_LEFT(metadata) = CONN_REQ_REPEAT; }
      else if(type == PIP_PKT_TEARDOWN) { TX_ATTEMPTS_LEFT(metadata) = TEAR_DOWN_REPEAT; }
      else { TX_ATTEMPTS_LEFT(metadata) = PIP_MAX_TX_ATTEMPTS; }
    } // End if(should_forward)
  } // End processRecdPkt()

  void acquireSpiResourceDone() {
    call Logger.logData(SPI_ACQ_DONE, 0);
    pip_state = PIP_S_DEFAULT;
    num_conn_req_sent = num_conn_req_recd = 0;
    call PIPTransmit.signalStartDone();
  } // End acquireSpiResourceDone()

  /*=============== End Various Functions ===============*/

  /**************** Init Commands ****************/
  command error_t Init.init() {
    m_p_rx_buf = &pipe_packet[0];
    return SUCCESS;
  }
  /*=============== End Init Commands ===============*/

  /**************** StdControl Commands ****************/

  void  resetPIPVars(){
      front = 0; rear = 0, q_len = 0;
      first_pkt_sent = FALSE;
      first_pkt_recd = FALSE;
      time_synced = FALSE;
      im_source=FALSE;
      init_txrad_state(PIP_S_CONFIG);
      init_rxrad_state(PIP_S_CONFIG);
      num_conn_req_sent = num_conn_req_recd=0;
      num_tear_down_recd=0;
      txfifo_state = rxfifo_state = FIFO_EMPTY;
      //holding_spi = FALSE;

      seed_init_done = FALSE;
      

      // Init PIPStats
      PIPStats.num_pkts_sent = 0;
      PIPStats.num_pkts_recd = PIPStats.num_crc_err = PIPStats. num_sim_err = 0;
      PIPStats.num_max_retry_exceeded = 0;
      PIPStats.num_data_pkts_recd = 0;
      PIPStats.num_eof_pkts_recd = PIPStats.num_eof_pkts_sent = 0;
      PIPStats.num_snack_pkts_recd = PIPStats.num_snack_pkts_sent = 0;
      PIPStats.num_frame_periods = PIPStats.num_queue_full = PIPStats.num_queue_empty = 0;

      pip_state = PIP_S_CONFIG;

      is_rxspi_in_progress=FALSE;
      is_txspi_in_progress=FALSE;
      
      /*SB: While resetting following  doesn't gurantee that u have changed
	the channel to default one  */
      curr_pip_channel = PIP_DEFAULT_CHANNEL;
  }

  
  command error_t StdControl.start() {
    atomic {
      reset_state();
      resetPIPVars();
      call InterruptFIFOP.enableFallingEdge();
      call CC2420Config.setAddressRecognition(TRUE, TRUE);
      call CC2420Config.setPanAddr(TOS_NODE_ID);
      call CC2420Config.setChannel(curr_pip_channel);
      call CC2420Config.sync(); 
    }
    return SUCCESS;
  } // End StdControl.start()

  command error_t StdControl.stop() {
    atomic {
      rx_m_state = RX_S_STOPPED;
      reset_state();
      call CSN.set();
      call InterruptFIFOP.disable();
    }
    return SUCCESS;
  } // End StdControl.stop()
  /*=============== End StdControl Commands ===============*/

  /**************** CC2420Receive Commands ****************/
  /**
   * Start frame delimiter signifies the beginning/end of a packet
   * See the CC2420 datasheet for details.
   */
  async command void CC2420Receive.sfd( uint32_t time ) {

    call Logger.logData(RAD_GOT_SFD, 0);
    if ( m_timestamp_size < TIMESTAMP_QUEUE_SIZE ) {
      uint8_t tail =  ( ( m_timestamp_head + m_timestamp_size ) % 
			TIMESTAMP_QUEUE_SIZE );
      m_timestamp_queue[ tail ] = time;
      m_timestamp_size++;
    }
  } // End CC2420Receive.sfd()

  async command void CC2420Receive.sfd_dropped() {
    if(m_timestamp_size > 0) { m_timestamp_size--; }
  } // End CC2420Receive.sfd_dropped()
  /*=============== End CC2420Receive Commands ===============*/

  /**************** PacketIndicator Commands ****************/
  command bool PacketIndicator.isReceiving() {
    bool receiving;
    atomic {
      receiving = prev_txrad_half_state.rxspi_initiated;
    }
    return receiving;
  }
  /*=============== End PacketIndicator Commands ===============*/  

  /**************** PIPReceive Commands ****************/
  async command void PIPReceive.reset(){
    call StdControl.stop();
    call Init.init();
    reset_state();
    resetPIPVars();
    pip_state = PIP_S_DEFAULT;
    call InterruptFIFOP.enableFallingEdge();
  }
  /** SB:Following command is used  if mote is completely dead
      eliminating chances of showing log ***/
  async command void PIPReceive.checkFIFOStatus(){
    if ( !call FIFO.get() && !call FIFOP.get() ) {
      call Leds.set(0);
    } else if ( !call FIFO.get() && call FIFOP.get() ) {
      call Leds.set(1);   
    } else   if ( call FIFO.get() && !call FIFOP.get() ) {
      call Leds.set(2);
    } else {
      call Leds.set(3);
    }    
  }
  /** Dump the stats into logging array**/
  async command void PIPReceive.connectionDone() {
    pip_state = PIP_S_DEFAULT;
    call Leds.set(7);
    call Logger.logData(STAT_NUM_PKTS_SENT, PIPStats.num_pkts_sent);
    call Logger.logData(STAT_NUM_PKTS_RECD, PIPStats.num_pkts_recd);
    call Logger.logData(STAT_NUM_CRC_ERR, PIPStats.num_crc_err);
    call Logger.logData(STAT_NUM_SIM_ERR, PIPStats.num_sim_err);
    call Logger.logData(STAT_NUM_ACK_TIMEOUT, PIPStats.num_ack_timeout);
    call Logger.logData(STAT_NUM_MAX_RET, PIPStats.num_max_retry_exceeded);
    call Logger.logData(STAT_NUM_FRAME_PERIODS, PIPStats.num_frame_periods);
    call Logger.logData(STAT_NUM_QUEUE_FULL, PIPStats.num_queue_full);
    call Logger.logData(STAT_NUM_QUEUE_EMPTY, PIPStats.num_queue_empty);
  } // End PIPReceive.connectionDone()

  async command void PIPReceive.InterruptFIFOP_disable() {
    call InterruptFIFOP.disable();
  } // End PIPReceive.InterruptFIFOP_disable()

  async command void PIPReceive.RxConnReqTimerFired() {
    cc2420_header_t *hdr;
    cc2420_metadata_t* metadata;
    if(m_p_rx_buf == NULL) { call Logger.logData(RXDEBUG, __LINE__); return; }
    hdr = (cc2420_header_t*)(m_p_rx_buf->header);
    metadata = (cc2420_metadata_t*)(m_p_rx_buf->metadata);
    call Logger.logData(CONN_REQ_TIMER, hdr->pip_hdr.datasrc);
    if(hdr->pip_hdr.datasrc == TOS_NODE_ID) {
      pip_state = PIP_S_RXRAD_HALF;
      call PIPTransmit.signalIncomingConnReq(m_p_rx_buf, hdr->length-LENGTH_FIELD_ADJ);
    } else {
      pip_state = PIP_S_TXCONN_REQ;
      // Pkt needs to be forwarded, add the received packet to the queue
      rear = (rear+1)%CIRC_QUEUE_ARRAY_SIZE; q_len++;
      // Re-adjust addresses in the packet
      hdr->dest = hdr->destpan = pip_prevNode;
      hdr->src = TOS_NODE_ID;
      hdr->fcf |= 1 << IEEE154_FCF_ACK_REQ;
      TX_ATTEMPTS_LEFT(metadata) = CONN_REQ_REPEAT;
    } // End else of if(hdr->pip_hdr.datasrc == TOS_NODE_ID)
    call Logger.logData(PIP_STATE_LOG, pip_state);
    call PIPReceive.checkAndSwitchToRx();
  } // End PIPReceive.RxConnReqTimerFired()

  /* Attempt to acquire the SPI bus to receive a packet.
     PIP: Assumes that m_p_rx_buf has already been set
   */
  async command void PIPReceive.tryReceiveSPI() { 

    /***In routing state if a tx_spi of a pkt is in progress, then
	ignre the recvd pkt. This is for phase1 of C2P.(broadcast pkts)**/
    if(pip_state==PIP_S_ROUTING && is_txspi_in_progress) {
      call Logger.logData(RXDEBUG, 0);
      return;
    }else {
      is_rxspi_in_progress=TRUE;
    }

    if(rx_m_state != RX_S_STARTED) {
      call Logger.logData(RXDEBUG, __LINE__);
    }

    rx_m_state = RX_S_RX_LENGTH;
    m_p_rx_buf = &pipe_packet[rear];
         
    call Logger.logData(SPI_RX_START, 0);
    prev_txrad_half_state.rxspi_initiated = TRUE;
    if(call PIPTransmit.acquireSpiResource() == SUCCESS) { receiveSPI(); }
    else { call Logger.logData(RXDEBUG, __LINE__); } // No deferred requests? (Bhaskar)
  } // End PIPReceive.tryReceiveSPI()

  /** check out cc2420 doc for proper reset seq**/
  async command void PIPReceive.resetFIFOP() {
    call InterruptFIFOP.disable();   
    call InterruptFIFOP.enableFallingEdge();
  }

  async command void PIPReceive.checkAndSwitchToRx() {
    bool radioDone, spiDone;
    
    if(!holding_spi) {
      call Logger.logData(RXDEBUG, __LINE__);
      rx_m_state = RX_S_FLUSH;
      if(call PIPTransmit.acquireSpiResource() != SUCCESS) { return; }
      rx_m_state = RX_S_STARTED;
    }
    // Here we are certainly holding SPI resource
    RXFIFOFlush();    
    
    /**** Do not call processRecdPkt b4 FIFOFLUSH as it might trigger
	  TXSPI of next pkt **/

    if(shouldProcessRecdPkt){
      if(pip_state==PIP_S_ROUTING){
	/** This is non-pipelined mode. No call to  switchToRxRadioMode() will be made. So do the necessary
	    steps here. Sequence is diff here as processRecdPkt may 	    
	    directly trigger txspi of another pkt. So make sure FIFOP is reset
	    for txspi **/
	radioDone=TRUE;
	spiDone=TRUE;      
	call PIPReceive.resetFIFOP();

      }
      atomic {
	processRecdPkt();
	shouldProcessRecdPkt=FALSE;
      }   
      if(pip_state==PIP_S_ROUTING){
	rx_m_state = RX_S_STARTED;
	init_rxrad_state(pip_state);
	call Logger.logData(PIP_STATE_LOG, pip_state);
	return;
      }
    }
           
    radioDone = !prev_txrad_half_state.pkt_transmitted || prev_txrad_half_state.radio_work_done;
    spiDone = !prev_txrad_half_state.rxspi_initiated || prev_txrad_half_state.spi_work_done;
    if(!radioDone || !spiDone) { return; }
    // Both parts of PIP_S_TXRAD_HALF are done at this stage

    call PIPTransmit.releaseSpiResource();
    if((pip_state == PIP_S_TXRAD_HALF) || (pip_state == PIP_S_RXRAD_HALF)) {
      if((TOS_NODE_ID == SINK) || first_pkt_sent) {
	if(time_synced) { call PIPTransmit.SetNextTxRTimer(); }
      }
    }
    
    call PIPReceive.switchToRxRadioMode();  // Can switch to RX
    

  } // End PIPReceive.checkAndSwitchToRx()
  

  async command void PIPReceive.switchToRxRadioMode() {
    uint8_t nextChannel = PIP_DEFAULT_CHANNEL;
    uint8_t nextState = pip_state;
    if((pip_state == PIP_S_RXRAD_HALF) || (pip_state == PIP_S_TXRAD_HALF)) {
      nextState = PIP_S_RXRAD_HALF;
      nextChannel = pip_myRxChannel;
      if(FLOW_CONTROL_ENABLED && (q_len >= CIRC_QUEUE_MAX_ELEMENTS)) {
	PIPStats.num_queue_full++;
	call Logger.logData(SPI_RX_QFULL, PIPStats.num_queue_full);
	nextChannel = PIP_DEFAULT_CHANNEL;
      }
    }
    if(pip_state!=PIP_S_ROUTING) { // for routing default channel is used
      call PIPTransmit.setChannel(nextChannel);
    }
    call PIPReceive.resetFIFOP();

    rx_m_state = RX_S_STARTED;
    init_rxrad_state(nextState);
    call Logger.logData(PIP_STATE_LOG, pip_state);

    call PIPTransmit.beginSPITransmit();
    
  } // End PIPReceive.switchToRxRadioMode()
  /*=============== PIPReceive Commands ===============*/

  /**************** Timer Events ****************/
  async event void Alarm.fired() {}
  /*=============== End Timer Events ===============*/
  
  /**************** InterruptFIFOP Events ****************/
  void InterruptFIFOP_fired_helper();
  async event void InterruptFIFOP.fired() {
    atomic { InterruptFIFOP_fired_helper(); }
  } // End InterruptFIFOP.fired()
  void InterruptFIFOP_fired_helper() {
    
   
    if(prev_txrad_half_state.expecting_ack_fifop) {
      // This FIFOP must correspond to an ACK received
      call Logger.logData(RAD_ACK_FIFOP, pip_state);
      atomic tx_m_state = TX_S_STARTED; // Additional check...
      prev_txrad_half_state.radio_work_done = TRUE;
      prev_txrad_half_state.expecting_ack_fifop = FALSE;
      //call PIPTransmit.signalSendDone(SUCCESS);
      call PIPReceive.checkAndSwitchToRx();
      return;
    }

    
    if(rx_m_state != RX_S_STARTED) {
      call Logger.logData(RXDEBUG, __LINE__);
      call Logger.logData(DEBUG, rx_m_state);
      return;
    }
    
    // RXFIFO should be empty to begin with
    if(rxfifo_state != FIFO_EMPTY) { call Logger.logData(RXDEBUG, __LINE__); }
    rxfifo_state = FIFO_FULL;



    // Now it is confirmed that we have rx_m_state = RX_S_STARTED
    // Assuming this to be a data packet
    prev_rxrad_half_state.pkt_received = TRUE;

    call Logger.logData(RAD_RX_DONE, 0);
    if(!first_pkt_recd) {
      first_pkt_recd = TRUE;
      //call PIPTransmit.startSendLogAlarm();
    }
    
    

    if((pip_state == PIP_S_DEFAULT) || (pip_state == PIP_S_RXCONN_REQ) ||
       (pip_state == PIP_S_TXCONN_REQ) || !time_synced) {
      // Do not wait for timer to fire to receive from RXFIFO
      if(q_len >= CIRC_QUEUE_MAX_ELEMENTS) { call Logger.logData(RXDEBUG, __LINE__); }
      else { call PIPReceive.tryReceiveSPI(); }
    }
  } // End InterruptFIFOP_fired_helper()
  /*=============== InterruptFIFOP Events ===============*/

  /**************** PIPTransmit Events ****************/
  async event void PIPTransmit.SpiResourceGranted() {
    call Logger.logData(RXDEBUG, __LINE__);
    if(pip_state == PIP_S_ACQ_SPI) {
      acquireSpiResourceDone();
    } else if(rx_m_state == RX_S_FLUSH) {
      rx_m_state = RX_S_STARTED;
      prev_txrad_half_state.spi_work_done = TRUE;
      //RXFIFOFlush(); // Flush RXFIFO will be done by checkAndSwitchToRx()
      call PIPReceive.checkAndSwitchToRx();
    } else {
      receiveSPI();
    }
  } // End PIPTransmit.SpiResourceGranted()
  /*=============== End PIPTransmit Events ===============*/
  
  /**************** RXFIFO Events ****************/
  /**
   * We received some bytes from the SPI bus.  Process them in the context
   * of the state we're in.  Remember the length byte is not part of the length
   */
  void RXFIFO_readDone_helper(uint8_t* rx_buf, uint8_t rx_len, error_t error);
  async event void RXFIFO.readDone(uint8_t* rx_buf, uint8_t rx_len, error_t error) {
    atomic { RXFIFO_readDone_helper(rx_buf, rx_len, error); }
  } // End RXFIFO.readDone()
  void RXFIFO_readDone_helper(uint8_t* rx_buf, uint8_t rx_len, error_t error) {
    cc2420_header_t* header = call CC2420PacketBody.getHeader( m_p_rx_buf );
    uint8_t tmpLen __DEPUTY_UNUSED__ = sizeof(message_t) - (offsetof(message_t, data) - sizeof(cc2420_header_t));
    uint8_t* COUNT(tmpLen) buf = TCAST(uint8_t* COUNT(tmpLen), header);
    
    //rxFrameLength = buf[ 0 ];

    if(rx_m_state == RX_S_RX_LENGTH) {

      rx_m_state = RX_S_RX_FCF;
      if(q_len >= CIRC_QUEUE_MAX_ELEMENTS) {
	PIPStats.num_queue_full++;
	call Logger.logData(SPI_RX_QFULL, PIPStats.num_queue_full);
	abortSPIRead();
	return;
      } // End if(queue full)

      header->length = rxFrameLength; // record the length onto the header
      if((rxFrameLength+1) > m_bytes_left) {
	call Logger.logData(RXDEBUG, __LINE__);
	call Logger.logData(DEBUG, rxFrameLength);
	call Logger.logData(DEBUG, m_bytes_left);
	abortSPIRead(); // Length of this packet is bigger than the RXFIFO, flush it out.
	return;
      }
      if(rxFrameLength > (MAC_PACKET_SIZE+sizeof(timesync_radio_t))) {
	call Logger.logData(RXDEBUG, __LINE__);
	abortSPIRead(); // Length is too large; we have to flush the entire RXFIFO
	return;
      }
      if(rxFrameLength == 0) {
	call Logger.logData(RXDEBUG, __LINE__);
	abortSPIRead(); // Length == 0; abort
	return;
      }
      if(rxFrameLength <= SACK_HEADER_LENGTH) {
	if(rxFrameLength == 5) {
	  // Must be an overheard ACK
	  call Logger.logData(RAD_OVERHEARD_ACK, rxFrameLength);
	} else {
	  call Logger.logData(RXDEBUG, __LINE__);
	  call Logger.logData(DEBUG, rxFrameLength);
	}
	abortSPIRead(); // This is really a bad packet, skip FCF and get it out of here
	return;
      }

      if ( !call FIFO.get() && !call FIFOP.get() ) {
	m_bytes_left -= rxFrameLength + 1;
      }

      // This packet has an FCF byte plus at least one more byte to read
      call RXFIFO.continueRead(buf + 1, SACK_HEADER_LENGTH);
      return;
    } // End if(rx_m_state == RX_S_RX_LENGTH)

    if(rx_m_state == RX_S_RX_FCF) {
      rx_m_state = RX_S_RX_PAYLOAD;
      call RXFIFO.continueRead(buf+1+SACK_HEADER_LENGTH, rxFrameLength-SACK_HEADER_LENGTH);
      return;
    } // End if(rx_m_state == RX_S_RX_FCF)

    if(rx_m_state == RX_S_RX_PAYLOAD) {
      rx_m_state = RX_S_STARTED;
      rxfifo_state = FIFO_EMPTY; // Done reading RXFIFO now

      // Not sure what this code does...
      if ( m_timestamp_size ) {
	if ( rxFrameLength > 10 ) {		
	  call PacketTimeStamp.set(m_p_rx_buf, m_timestamp_queue[ m_timestamp_head ]);
	  m_timestamp_head = ( m_timestamp_head + 1 ) % TIMESTAMP_QUEUE_SIZE;
	  m_timestamp_size--;
	}
      } else {
	call PacketTimeStamp.clear(m_p_rx_buf);
      }

      // buf[rxFrameLength] >> 7 checks the CRC
      if ( ( buf[ rxFrameLength ] >> 7 ) && rx_buf ) {
	uint8_t type = ( header->fcf >> IEEE154_FCF_FRAME_TYPE ) & 7;
	if(type != IEEE154_TYPE_DATA) {
	  call Logger.logData(RXDEBUG, __LINE__);
	}

	//processRecdPkt();
	shouldProcessRecdPkt=TRUE;
	signal CC2420Receive.receive(type, m_p_rx_buf);
      } else {
	call Logger.logData(RAD_RX_CRCERR, 0);
	prev_txrad_half_state.rxspi_crc_error = TRUE; // CRC error

	//processRecdPkt(); 
	shouldProcessRecdPkt=TRUE;
      }

      prev_txrad_half_state.spi_work_done = TRUE;
      call PIPReceive.checkAndSwitchToRx();
      return;
    } // End if(rx_m_state == RX_S_RX_PAYLOAD)

    call Logger.logData(RXDEBUG, __LINE__); // Should never be here, unknown state
  } // End RXFIFO_readDone_helper()

  async event void RXFIFO.writeDone( uint8_t* tx_buf, uint8_t tx_len, error_t error ) {
  }  
  /*=============== End RXFIFO Events ===============*/

  /**************** CC2420Config Events ****************/
  void CC2420Config_syncDone_helper(error_t error);
  event void CC2420Config.syncDone(error_t error) {
    atomic { CC2420Config_syncDone_helper(error); }
  } // End CC2420Config.syncDone()
  void CC2420Config_syncDone_helper(error_t error) {
    if(pip_state != PIP_S_CONFIG) { call Logger.logData(RXDEBUG, __LINE__); }
    pip_state = PIP_S_ACQ_SPI;
    if(call PIPTransmit.acquireSpiResource() == SUCCESS) {
      acquireSpiResourceDone();
    }
  } // End CC2420Config_syncDone_helper()
  /*=============== End CC2420Config Events ===============*/

  /***SB : TimeSync functionality ***/
  void eventTime(message_t* msg){
    uint32_t sender_timestamp, received_timestamp;
    atomic {
      timesync_radio_t* timesync = getFooter(msg);
      sender_timestamp=(uint32_t)(*timesync) ; // TimeStamp received in pkt
      received_timestamp=call PacketTimeStamp32khz.timestamp(msg); // Time at which pkt is recvd
      if(!call PacketTimeStamp32khz.isValid(msg)) {
	call Logger.logData(RXDEBUG, __LINE__);
	return;
      }
      node_time_offset=sender_timestamp-received_timestamp;
      if(!time_synced) { call Logger.logData(ADJ_TIMER, node_time_offset); }
      time_synced = TRUE;
    }
    //return node_time_offset;
  }
  inline void* getFooter(message_t* msg){
    return msg->data + payloadLength(msg);
  }
  uint8_t payloadLength(message_t* msg)
  {
    return (call CC2420PacketBody.getHeader(msg))->length - CC2420_SIZE - sizeof(timesync_radio_t);
  }
  /*========End TimeSync functionality=====*/

} // End implementation
