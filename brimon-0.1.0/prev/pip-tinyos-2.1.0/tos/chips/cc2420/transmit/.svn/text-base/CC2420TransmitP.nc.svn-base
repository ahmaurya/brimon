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
 * @author Jung Il Choi Initial SACK implementation
 * @version $Revision: 1.9 $ $Date: 2008/07/11 19:21:23 $
 */

#include "CC2420.h"
#include "CC2420TimeSyncMessage.h"
#include "crc.h"
#include "message.h"


/* PIP */
#include "pipeline.h"

module CC2420TransmitP @safe() {

  provides interface Init;
  provides interface StdControl;
  provides interface CC2420Transmit as Send;
  provides interface RadioBackoff;
  provides interface ReceiveIndicator as EnergyIndicator;
  provides interface ReceiveIndicator as ByteIndicator;

  /***************************************/
  provides interface PIPTransmit;
  provides interface PIPMac;
  uses interface Random;
  uses interface ParameterInit<uint16_t> as SeedInit;
  /***************************************/

  uses interface Alarm<T32khz,uint32_t> as TxRTimer;
  uses interface CC2420Packet;
  uses interface CC2420PacketBody;
  uses interface PacketTimeStamp<T32khz,uint32_t>;
  uses interface PacketTimeSyncOffset;
  uses interface GpioCapture as CaptureSFD;
  uses interface GeneralIO as CCA;
  uses interface GeneralIO as CSN;
  uses interface GeneralIO as SFD;

  uses interface Resource as SpiResource;
  uses interface ChipSpiResource;
  uses interface CC2420Fifo as TXFIFO;
  uses interface CC2420Ram as TXFIFO_RAM;
  uses interface CC2420Register as TXCTRL;
  uses interface CC2420Strobe as SNOP;
  uses interface CC2420Strobe as STXON;
  uses interface CC2420Strobe as STXONCCA;
  uses interface CC2420Strobe as SFLUSHTX;
  uses interface CC2420Register as MDMCTRL1;
  uses interface CC2420Register as FSCTRL;
  uses interface CC2420Strobe as SRXON;
  uses interface CC2420Strobe as SRFOFF;

  uses interface CC2420Receive;
  uses interface Leds;

  /***************************/
  uses interface Alarm<T32khz,uint32_t> as sendLogAlarm;

    

  uses interface PIPReceive;
  uses interface ReceiveIndicator as SPIReceiveIndicator;
  uses interface CC2420Config;
  uses interface PacketAcknowledgements;

  uses interface Logger;
  uses interface Routing;

  uses interface BusyWait<TMicro,uint16_t> as BusyWait;
  /***************************/
}

implementation {

  // This specifies how many jiffies the stack should wait after a
  // TXACTIVE to receive an SFD interrupt before assuming something is
  // wrong and aborting the send. There seems to be a condition
  // on the micaZ where the SFD interrupt is never handled.
  enum {
    CC2420_ABORT_PERIOD = 320
  };

  /**************** Variables ****************/
  
  /** This is the message currently being sent */
  norace message_t * ONE_NOK m_msg;
  norace uint8_t m_tx_power;

  /* This is used within CaptureSFD.captured() to detect if an SFD
     fired while still processing the previous SFD */
  bool m_receiving = FALSE;

  /** Record the previous time SFD was captured.  This is useful to
      detect more than one SFD interrupt while in the handler. */
  uint16_t m_prev_sfd_capture_time;

  /** Byte reception/transmission indicator.  This variable is
      set/reset in the captured() function. */
  bool sfdHigh;
  /*=============== End Variables ===============*/

  /**************** Function Prototypes ****************/
  error_t send(message_t *p_msg);
  void loadTXFIFO();
  void attemptRadioSend();
  void signalSendDone(error_t err);
  /*=============== End Function Prototypes ===============*/

  /**************** Various functions ****************/
  /** Convert from 16-bit time to 32-bit time, assuming we're not too
      much off from 'recent_time' */
  inline uint32_t time16to32(uint16_t time, uint32_t recent_time) {
    if ((recent_time&0xFFFF)<time)
      return ((recent_time-0x10000UL)&0xFFFF0000UL)|time;
    else
      return (recent_time&0xFFFF0000UL)|time;
  } // End time16to32

  /**
   * Set up a message to be sent. Acquire SPI resource and call
   * loadTXFIFO() which will load the packet onto TXFIFO.
   * @param *p_msg Pointer to the message that needs to be sent
   * @param cca TRUE if this transmit should use clear channel assessment
   */
  error_t send(message_t* ONE p_msg) {
    if((pip_state == PIP_S_CONFIG) || (pip_state == PIP_S_ACQ_SPI)) {
      call Logger.logData(TXDEBUG, __LINE__);
      return FAIL;
    }
    if(tx_m_state != TX_S_STARTED) {
      call Logger.logData(TXDEBUG, __LINE__);
      call Logger.logData(TXDEBUG, tx_m_state);
    }

    if(pip_state==PIP_S_ROUTING && is_rxspi_in_progress) {
      /** The upper layer layer will retry after random amt of time**/
      /** Routing pkts are supposed to be CSMA based. However,
	  implementing CSMA, in addtion to tdma,  would complicate the MAC. so
	  preferred is application level randomness**/
      call Logger.logData(RTG_BCAST_SPI_CONGESTION, __LINE__);
      
      return FAIL;;
    }else {
      is_txspi_in_progress=TRUE;
    }

   
    atomic tx_m_state = TX_S_LOAD;
    m_msg = p_msg;
    if(call PIPTransmit.acquireSpiResource() == SUCCESS) {  loadTXFIFO(); }
    /* else loadTXFIFO() will be called from SpiResource.granted() */
    return SUCCESS;
  } // End send()

  /**
   * Attempt to send the packet we have loaded into the tx buffer on 
   * the radio chip. 
   * the channel is clear.  If we're not concerned about whether or not
   * the channel is clear (i.e. m_cca == FALSE), then STXON will send the
   * packet without checking for a clear channel.  For PIP, no CCA is
   * done and m_cca has been deleted from the code (Bhaskar).
   *
   * If the packet didn't get sent, then congestion == TRUE.  In that case,
   * we reset the backoff timer and try again in a moment.  For PIP,
   * there is no CCA, no backoff (Bhaskar).
   *
   * If the packet got sent, we should expect an SFD interrupt to take
   * over, signifying the packet is getting sent.
   */
  void attemptRadioSend() {
    uint8_t status;
    prev_txrad_half_state.txrad_pkt_type = prev_rxrad_half_state.txspi_pkt_type;
    if(prev_rxrad_half_state.txspi_pkt_type == PIP_PKT_CONN_REQ) {
      call BusyWait.wait(CONN_REQ_GAP);
      call PIPTransmit.setChannel(PIP_DEFAULT_CHANNEL);
      call Logger.logData(TX_CONN_REQ, num_conn_req_sent);
    } 
    else if(pip_state==PIP_S_ROUTING) {
      /** Routing pkts are always sent & recvd over default channel**/
      call PIPTransmit.setChannel(PIP_DEFAULT_CHANNEL);
    } 

    /** SB : SNACK travels in reverse dir**/
    else if(prev_rxrad_half_state.txspi_pkt_type ==PIP_PKT_SNACK ){
      call PIPTransmit.setChannel(pip_prevNodeRxChannel);
    }    
    else {
      call PIPTransmit.setChannel(pip_nextNodeRxChannel);
    }

    call Logger.logData(RAD_TX_STROBE, curr_pip_channel);   
    call CSN.clr();
    status = call STXON.strobe(); // Note: no CCA done, and no congn should happen in PIP
    prev_txrad_half_state.pkt_transmitted = TRUE;
    atomic tx_m_state = TX_S_SFD;
    call CSN.set();
  } // End attemptRadioSend()

  /** 
   * Setup the packet transmission power and load the tx fifo buffer on
   * the chip with our outbound packet.  
   *
   * Warning: the tx_power metadata might not be initialized and
   * could be a value other than 0 on boot.  Verification is needed here
   * to make sure the value won't overstep its bounds in the TXCTRL register
   * and is transmitting at max power by default.
   *
   * It should be possible to manually calculate the packet's CRC here and
   * tack it onto the end of the header + payload when loading into the TXFIFO,
   * so the continuous modulation low power listening strategy will continually
   * deliver valid packets.  This would increase receive reliability for
   * mobile nodes and lossy connections.  The crcByte() function should use
   * the same CRC polynomial as the CC2420's AUTOCRC functionality.
   */
  void loadTXFIFO() {
    cc2420_header_t* header = (cc2420_header_t*)(m_msg->header);
    pip_header_t *piphdr = (pip_header_t*)(&(header->pip_hdr));//(pip_header_t*)(m_msg->data);
    cc2420_metadata_t *metadata = (cc2420_metadata_t*)(m_msg->metadata);
    uint8_t tx_power = metadata->tx_power;
    uint8_t tmpLen __DEPUTY_UNUSED__ = header->length;
    uint16_t err_rate_factor = 1;

    prev_rxrad_half_state.txspi_initiated = TRUE;
    call Logger.logData(SPI_TX_START, header->length);
    prev_rxrad_half_state.txspi_pkt_type = piphdr->pip_pkt_type;
    // Decrement the number of transmission attempts left for this pkt
    TX_ATTEMPTS_LEFT(metadata)--;

    // Simulate CRC errors with the configured percentage
    PIPStats.num_pkts_sent++;
    // Simulate higher error rate for some node
    //if(TOS_NODE_ID == 2) { err_rate_factor = 4; }
    if( expt_or_log  &&
       (call Random.rand16() < (uint16_t)(err_rate_factor*SIM_ERR_FRAC_UINT16))) {
      PIPStats.num_sim_err++;
      // Simulating errors by sending into a blackhole PAN address
      header->destpan = BLACKHOLE_PAN;
    } else {
      // SB:This is needed explicitly since otherwise all retx
      //  get destined to BLACKHOLE_PAN in case first tx is destined to BLACKHOLE_PAN
      header->destpan = header->dest;
    }

    // Request hardware ACK
    //call PacketAcknowledgements.requestAck(m_msg);
    header->fcf |= 1 << IEEE154_FCF_ACK_REQ;
    if( (piphdr->pip_pkt_type == PIP_PKT_CONN_REQ) ||
        (piphdr->pip_pkt_type == PIP_PKT_TEARDOWN) ||
        (piphdr->pip_pkt_type == RTG_HELLOMSG)     ||
        (piphdr->pip_pkt_type == RTG_RESET_STATE) ) {
      header->fcf &= ~(1 << IEEE154_FCF_ACK_REQ); // no ack for conn req & teardown
    }
    header->fcf |= ( ( IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE ) |
		     ( 1 << IEEE154_FCF_INTRAPAN ) |
  		     ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE ) |
		     ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) );
    if ( !tx_power ) {
      tx_power = CC2420_DEF_RFPOWER;
    }
    
    call CSN.clr();
    if ( m_tx_power != tx_power ) {
      call TXCTRL.write( ( 2 << CC2420_TXCTRL_TXMIXBUF_CUR ) |
			 ( 3 << CC2420_TXCTRL_PA_CURRENT ) |
			 ( 1 << CC2420_TXCTRL_RESERVED ) |
			 ( (tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL ) );
    }
    
    m_tx_power = tx_power;    
      

      
    call TXFIFO.write(TCAST(uint8_t * COUNT(tmpLen), header), header->length+1-MAC_FOOTER_SIZE);
    
  } // End loadTXFIFO()

  /* Signal the higher layer after checking if further retransmissions
     are needed */
  void signalSendDone(error_t err) {
    message_t *p_msg = m_msg;
    cc2420_header_t *header = (cc2420_header_t*)(&(p_msg->header));
    pip_header_t *piphdr = &(header->pip_hdr);
    cc2420_metadata_t *metadata = (cc2420_metadata_t*)(&(p_msg->metadata));
    bool should_retransmit = (TX_ATTEMPTS_LEFT(metadata) > 0);
    call Logger.logData(RAD_TX_DONE, TX_ATTEMPTS_LEFT(metadata));
    // Need to handle some retransmissions here
    // For connection request pkts and teardown pkts, retransmissions
    // are required irrespective of whether or not the previous
    // transmission was successful
    // For other pkt types, retransmissions are required only if the
    // previous transmission was not acked


    if((piphdr->pip_pkt_type != PIP_PKT_CONN_REQ) &&
       (piphdr->pip_pkt_type != PIP_PKT_TEARDOWN)) {
      should_retransmit = (should_retransmit && (err != SUCCESS));
      if(TX_ATTEMPTS_LEFT(metadata) == 0) { PIPStats.num_max_retry_exceeded++; }
    }
    if(piphdr->pip_pkt_type == PIP_PKT_CONN_REQ) { num_conn_req_sent++; }
    if(piphdr->pip_pkt_type == PIP_PKT_TEARDOWN) { num_tear_down_sent++; }


    if(should_retransmit) {
      // Nothing else to do for now
      // Note: m_msg here will be non NULL
      return;
    }

    m_msg = NULL; // Done with this pkt



    /*** If this was routing msg, signal senddone & return */
    if(piphdr->pip_pkt_type == RTG_HELLOMSG) {
      signal PIPMac.sendHelloMsgDone(p_msg,err);      
      return ;
    } else if(piphdr->pip_pkt_type == RTG_RESET_STATE) {
      signal PIPMac.sendRtgResetMsgDone(p_msg,err);      
      return ;
    } else if(piphdr->pip_pkt_type == RTG_LSQUERYMSG) {
      signal PIPMac.sendLSQueryMsgDone(p_msg,err);      
      return ;
    } else if(piphdr->pip_pkt_type == RTG_LSRESPONSEMSG) {
      signal PIPMac.sendLSResponseMsgDone(p_msg,err);      
      return ;
    }

    // Do the appropriate state change
    if(piphdr->pip_pkt_type == PIP_PKT_CONN_REQ) {
      pip_state = PIP_S_RXRAD_HALF;
    } else if(piphdr->pip_pkt_type == PIP_PKT_TEARDOWN) {
      call PIPReceive.connectionDone(); // state change will be done within this
    } // else: no other state change


    
    // For src/dest nodes, do signal to higher layer; for
    // intermediate, delete pkt from queue
    // Note: some of the signal events below may cause m_msg to become non-null
    if(TOS_NODE_ID == piphdr->datasrc) { // data source node
      if(piphdr->pip_pkt_type == PIP_PKT_DATA) {
	signal PIPMac.sendDataDone(p_msg, err);
      } else if(piphdr->pip_pkt_type == PIP_PKT_EOF) {
	signal PIPMac.sendEOFDone(p_msg, err);
      } else if(piphdr->pip_pkt_type == PIP_PKT_TEARDOWN) {
	call Logger.logData(PIP_STATE_LOG, pip_state);
	signal PIPMac.sendTeardownDone(p_msg, err);
      } else {
	call Logger.logData(TXDEBUG, __LINE__);
      }
    } else if(TOS_NODE_ID == piphdr->datadest) { // data dest node
      if(piphdr->pip_pkt_type == PIP_PKT_DATA) {
	signal PIPMac.sendDataDone(p_msg, err);
      }else if(piphdr->pip_pkt_type == PIP_PKT_CONN_REQ) {
	call Logger.logData(PIP_STATE_LOG, pip_state);
	signal PIPMac.sendConnRequestDone(p_msg, err);
      } else if(piphdr->pip_pkt_type == PIP_PKT_SNACK) {
	signal PIPMac.sendSNACKDone(p_msg, err);
      } else {
	call Logger.logData(TXDEBUG, __LINE__);
      }
    }else { // intermediate node
      if(q_len <= 0) { call Logger.logData(TXDEBUG, __LINE__); }
      else { front = (front+1)%CIRC_QUEUE_ARRAY_SIZE; q_len--; } // Delete from queue
    }

    if(piphdr->pip_pkt_type == PIP_PKT_TEARDOWN && TOS_NODE_ID!=SINK) {
      /*** reset the PIPMac after teardown is sent, except for the
	   SINK (sink can never send teardown)**/
      call PIPTransmit.setChannel(PIP_DEFAULT_CHANNEL);
      call PIPMac.reset(PIP_S_DEFAULT);
    }

  } // End signalSendDone()

  /*=============== End various functions ===============*/
  
  /**************** Init Commands ****************/
  command error_t Init.init() {
    call CCA.makeInput();
    call CSN.makeOutput();
    call SFD.makeInput();
    return SUCCESS;
  }
  /*=============== End Init Commands ===============*/

  /**************** StdControl Commands ****************/

  command error_t StdControl.start() {
     atomic {
      call CaptureSFD.captureRisingEdge();
      atomic tx_m_state = TX_S_STARTED;
      m_receiving = FALSE;
      m_tx_power = 0;
      m_msg = NULL;
     }
     //call PIPTransmit.startSendLogAlarm();
     return SUCCESS;
  } // End StdControl.start()



  command error_t StdControl.stop() {
    atomic {
      atomic tx_m_state = TX_S_STOPPED;
      call TxRTimer.stop();
      call CaptureSFD.disable();
      call PIPTransmit.releaseSpiResource();
      call CSN.set();
    }
    return SUCCESS;
  } // End StdControl.stop()




  /*=============== End StdControl Commands ===============*/

  /**************** Send Commands ****************/
  error_t Send_send_helper(message_t* ONE p_msg, bool useCca);
  async command error_t Send.send(message_t* ONE p_msg, bool useCca) {
    atomic { return Send_send_helper(p_msg, useCca); }
  } // End Send.send()
  // XXXX: This will have to be augmented in PIPMac
  error_t Send_send_helper(message_t* ONE p_msg, bool useCca) {
    useCca = FALSE; // Ignore
    m_msg = p_msg;

    // PIP: send() will be called from beginSPITransmit()
    return SUCCESS;
    //return send( m_msg, useCca );
  } // End Send_send_helper()

  async command error_t Send.resend(bool useCca) {
    return FAIL;
  } // End Send.resend()

  async command error_t Send.cancel() {
    // Send.cancel not supported in PIP
    return FAIL;
  } // End Send.cancel()

  async command error_t Send.modify(uint8_t offset, uint8_t* buf, uint8_t len) {
    call CSN.clr();
    call TXFIFO_RAM.write( offset, buf, len );
    call CSN.set();
    return SUCCESS;
  } // End Send.modify()

  /*=============== End Send Commands ===============*/
  
  /***************** Indicator Commands ****************/
  command bool EnergyIndicator.isReceiving() {
    return !(call CCA.get());
  }
  
  command bool ByteIndicator.isReceiving() {
    bool high;
    atomic high = sfdHigh;
    return high;
  }  
  /*=============== End Indicator Commands ===============*/
  
  /**************** RadioBackoff Commands (These are not required for
 PIP) ****************/

  /**
   * Must be called within a requestInitialBackoff event
   * @param backoffTime the amount of time in some unspecified units to backoff
   */
  async command void RadioBackoff.setInitialBackoff(uint16_t backoffTime) {
    // No backoff in PIP (Bhaskar)
  }
  
  /**
   * Must be called within a requestCongestionBackoff event
   * @param backoffTime the amount of time in some unspecified units to backoff
   */
  async command void RadioBackoff.setCongestionBackoff(uint16_t backoffTime) {
    // No backoff in PIP (Bhaskar)
  }
  
  async command void RadioBackoff.setCca(bool useCca) {
    // No CCA in PIP (Bhaskar)
  }
  /*=============== End RadioBackoff Commands ===============*/
  
  inline void header_fill_helper(message_t *p_msg, bool data_direction, pip_pkt_type_t type, uint8_t payload_length) {
    cc2420_header_t *hdr = (cc2420_header_t*)(p_msg->header);
    cc2420_metadata_t *metadata = (cc2420_metadata_t*)(p_msg->metadata);
    pip_header_t *piphdr = &(hdr->pip_hdr);
    
    if(type != RTG_LSQUERYMSG && type != RTG_LSRESPONSEMSG) {
      /* for lsquery & lsresponse msgs dest was set by  routing module*/
      hdr->destpan = hdr->dest = data_direction ? pip_nextNode : pip_prevNode;
    }
    hdr->src = TOS_NODE_ID;
    piphdr->pip_pkt_type = type;
    piphdr->datadest = SINK;    
    if(TOS_NODE_ID == SINK ) {
      /** The conn_req, SNACK pkts will have src field set as SRC of
	  conn (not sink)**/
      atomic piphdr->datasrc = SRC;
    }else{ // This is source node
      piphdr->datasrc = TOS_NODE_ID;
    }
    if(type == PIP_PKT_CONN_REQ) { TX_ATTEMPTS_LEFT(metadata) = CONN_REQ_REPEAT; }
    else if(type == PIP_PKT_TEARDOWN) { TX_ATTEMPTS_LEFT(metadata) = TEAR_DOWN_REPEAT; }
    else if(type ==RTG_HELLOMSG || type==RTG_RESET_STATE){
      /** hello msgs & routing reset msgs are sent only once**/
      TX_ATTEMPTS_LEFT(metadata)=1;
      hdr->destpan= hdr->dest=AM_BROADCAST_ADDR;  
    } else if(type == RTG_LSQUERYMSG && type == RTG_LSRESPONSEMSG) {
      /** These are also sent only once**/
      TX_ATTEMPTS_LEFT(metadata)=1;
    } else { TX_ATTEMPTS_LEFT(metadata) = PIP_MAX_TX_ATTEMPTS; }
    hdr->length = payload_length + LENGTH_FIELD_ADJ;
  } // End header_fill_helper()

  /**************** PIPMac commands ****************/
  void PIPMac_reset_helper(pip_state_t local_pip_state);
  async command void PIPMac.reset(pip_state_t local_pip_state){
    PIPMac_reset_helper(local_pip_state);
  }
  void PIPMac_reset_helper(pip_state_t local_pip_state){
    call Leds.set(0);
    call PIPTransmit.reset();
    call PIPReceive.reset(); 
    call Logger.reset();
    //call PIPTransmit.startSendLogAlarm();
    pip_state=local_pip_state;
    call Logger.logData(PIPMAC_RESET,pip_state);
    signal PIPMac.resetDone();
  }
    
  async command error_t PIPMac.sendConnRequest(message_t *p_msg, uint8_t payload_length) {
    ConnPktPayload_t *payloadptr= (ConnPktPayload_t*) p_msg->data;


    /** check out the pip state diagram**/
    if((pip_state != PIP_S_DEFAULT) && (pip_state != PIP_S_TXCONN_REQ)) {
      call Logger.logData(TXDEBUG, __LINE__);
      call Logger.logData(PIP_STATE_LOG, pip_state);
      return FAIL;
    }
   
    if(expt_or_log) { 
      payloadptr->connreq_or_logquery=88; //check the header file
    } else {
      payloadptr->connreq_or_logquery=99;
    }
    call Routing.fillPathAndChannelInfo(SRC,payloadptr);
    
    my_depth=0;// already initialized to 0, just making sure..
    
    pip_myRxChannel=payloadptr->channelInfo[payloadptr->noHopsToGo];
    payloadptr->noHopsToGo--;
    pip_prevNode=payloadptr-> path[payloadptr->noHopsToGo];  // prev=> towards SRC
    pip_prevNodeRxChannel=payloadptr-> channelInfo[payloadptr->noHopsToGo]; 

    header_fill_helper(p_msg, FALSE, PIP_PKT_CONN_REQ, payload_length);

    m_msg = p_msg;
    pip_state = PIP_S_TXCONN_REQ;

    call Logger.logData(PIP_STATE_LOG, pip_state);
    return send(p_msg);
  } // End PIPMac.connRequest()






  async command error_t PIPMac.sendData(message_t *p_msg, uint8_t payload_length) {
    
    header_fill_helper(p_msg, TRUE, PIP_PKT_DATA, payload_length);
    m_msg = p_msg;
      
    return SUCCESS; // PIP: send() will be called from beginSPITransmit()
   
  } // End PIPMac.sendData()

  async command error_t PIPMac.sendEOF(message_t *p_msg, uint8_t payload_length) {
    PIPStats.num_eof_pkts_sent++;
    header_fill_helper(p_msg, TRUE, PIP_PKT_EOF, payload_length);
    m_msg = p_msg;
    // PIP: send() will be called from beginSPITransmit()
    return SUCCESS;
  } // End PIPMac.sendEOF()

  async command error_t PIPMac.sendSNACK(message_t *p_msg, uint8_t payload_length) {
    PIPStats.num_snack_pkts_sent++;
    header_fill_helper(p_msg, FALSE, PIP_PKT_SNACK, payload_length);
    m_msg = p_msg;
    // PIP: send() will be called from beginSPITransmit()
    return SUCCESS;
  } // End PIPMac.sendSNACK()

  async command error_t PIPMac.sendTeardown(message_t *p_msg, uint8_t payload_length) {
    header_fill_helper(p_msg, TRUE, PIP_PKT_TEARDOWN, payload_length);
    m_msg = p_msg;
    atomic num_tear_down_sent = 0; // reset the counter; needs to be sent TEAR_DOWN_REPEAT times
    // PIP: send() will be called from beginSPITransmit()
    return SUCCESS;
  } // End PIPMac.sendTeardown()

  async command error_t PIPMac.sendHelloMsg(message_t *p_msg, uint8_t payload_length) {
    pip_state=PIP_S_ROUTING;
    header_fill_helper(p_msg, FALSE, RTG_HELLOMSG, payload_length);
    m_msg = p_msg;
    return send(p_msg);
  } // End PIPMac.connRequest()

  async command error_t PIPMac.sendLSQueryMsg(message_t *p_msg, uint8_t payload_length){

    if(pip_state!=PIP_S_ROUTING) {
      call Logger.logData(TXDEBUG, pip_state);
    }
    header_fill_helper(p_msg, FALSE, RTG_LSQUERYMSG, payload_length);
    m_msg = p_msg;
    return send(p_msg);

  }
  async command error_t PIPMac.sendLSResponseMsg(message_t *p_msg, uint8_t payload_length){
    if(pip_state!=PIP_S_ROUTING) {
      call Logger.logData(TXDEBUG, pip_state);
    }

    header_fill_helper(p_msg, FALSE, RTG_LSRESPONSEMSG, payload_length);
    m_msg = p_msg;
    return send(p_msg);
  }

  async command error_t PIPMac.sendRtgResetMsg(message_t *p_msg, uint8_t payload_length){
    pip_state=PIP_S_ROUTING;
    header_fill_helper(p_msg, FALSE,  RTG_RESET_STATE, payload_length);
    m_msg = p_msg;
    return send(p_msg);
  }


  /*=============== End PIPMac commands ===============*/

  /**************** PIPTransmit commands ****************/
  void PIPTransmit_reset_helper();

  async command void PIPTransmit.reset(){
    PIPTransmit_reset_helper();
  }
  void PIPTransmit_reset_helper(){
    call StdControl.stop();
    call Init.init();
    call StdControl.start();
  }
  
  async command void PIPTransmit.TxRTimer_start(uint32_t timerval) {
    call TxRTimer.start(timerval);
  } // End PIPTransmit.TxRTimer_start()
  async command void PIPTransmit.TxRTimer_stop() {
    call TxRTimer.stop();
  } // End PIPTransmit.TxRTimer_stop()
  async command bool PIPTransmit.TxRTimer_isRunning() {
    return (call TxRTimer.isRunning());
  } // End PIPTransmit.TxRTimer_isRunning()
  

  async command void PIPTransmit.signalStartDone() {
    signal PIPMac.startDone();
  } // End PIPTransmit.signalStartDone()
  async command void PIPTransmit.signalSendDone(error_t status) {
    signalSendDone(status);
  } // End PIPTransmit.signalSendDone()

  async command void PIPTransmit.signalIncomingConnReq(message_t *p_msg, uint8_t len) {
    signal PIPMac.incomingConnRequest(p_msg, len);
  }
  async command void PIPTransmit.signalIncomingData(message_t *p_msg, uint8_t len) {
    signal PIPMac.incomingData(p_msg, len);
  }
  async command void PIPTransmit.signalIncomingEOF(message_t *p_msg, uint8_t len) {
    signal PIPMac.incomingEOF(p_msg, len);
  }
  async command void PIPTransmit.signalIncomingSNACK(message_t *p_msg, uint8_t len) {
    signal PIPMac.incomingSNACK(p_msg, len);
  }
  async command void PIPTransmit.signalIncomingTeardown(message_t *p_msg, uint8_t len) {
    signal PIPMac.incomingTeardown(p_msg, len);
  }
 
  async command void PIPTransmit.signalIncomingHelloMsg(message_t *p_msg, uint8_t len) {
    signal PIPMac.incomingHelloMsg(p_msg, len);
  }
 
  async command void PIPTransmit.signalIncomingLSQueryMsg(message_t *p_msg, uint8_t len) {
    signal PIPMac.incomingLSQueryMsg(p_msg, len);
  }
  async command void PIPTransmit.signalIncomingLSResponseMsg(message_t *p_msg, uint8_t len) {
    signal PIPMac.incomingLSResponseMsg(p_msg, len);
  }
  async command void PIPTransmit.signalIncomingRtgResetMsg(message_t *p_msg, uint8_t len) {
    signal PIPMac.incomingRtgResetMsg(p_msg, len);
  }
 

  /** func used to set channel**/
  async command void PIPTransmit.setChannel(uint8_t channel) {
    if(channel == curr_pip_channel) { return; }
    call Logger.logData(SWITCH_CH, channel);
    if(!holding_spi) {
      call Logger.logData(TXDEBUG, __LINE__);
      return;
    }
    call CSN.set();
    call CSN.clr();
    call SRFOFF.strobe();
    call FSCTRL.write( ( 1 << CC2420_FSCTRL_LOCK_THR ) |
          ( ( (channel - 11)*5+357 ) << CC2420_FSCTRL_FREQ ) );
    call CSN.set();
    call CSN.clr();
    call SRXON.strobe();
    call CSN.set();
    curr_pip_channel = channel;
    call Logger.logData(AFTER_SWITCH_CH, channel);
  } // End PIPTransmit.setChannel()

  async command error_t PIPTransmit.acquireSpiResource() {
    error_t status;
    if(holding_spi) { return SUCCESS; }
    if(call SpiResource.isOwner()) { holding_spi = TRUE; return SUCCESS; }
    status = call SpiResource.immediateRequest();
    if(status == SUCCESS) { holding_spi = TRUE; }
    else { call SpiResource.request(); }
    return status;
  } // End PIPTransmit.acquireSpiResource()

  /** In PIP, there is no release of SPI resource, since the radio
      continuously needs it */
  async command error_t PIPTransmit.releaseSpiResource() {
    if(TRUE) { return SUCCESS; }
    holding_spi = FALSE;
    call SpiResource.release();
    return SUCCESS;
  } // End PIPTransmit.releaseSpiResource()

  /** Function called to begin the next transfer to TXFIFO operation */
  // XXXX: state checking
  async command void PIPTransmit.beginSPITransmit() {
    if(pip_state == PIP_S_TXRAD_HALF) { call Logger.logData(TXDEBUG, __LINE__); return; }
    if(m_msg != NULL) { send(m_msg); } // pkt needs (re)transmission (?)
    else {
      if((TOS_NODE_ID != SINK) && !im_source) { // intermediate node
	if(q_len > 0) { send(&pipe_packet[front]); }
	else {
	  PIPStats.num_queue_empty++;
	  call Logger.logData(SPI_TX_QEMPTY, PIPStats.num_queue_empty);
	}
      }
    }
  } // End PIPTransmit.beginSPITransmit()

  /* Should set the TxrTimer to the next appropriate half frame
     boundary + guard time.  Note how the calculation differs subtly
     for odd and even nodes. */
  // XXXX: state checking
  async command void PIPTransmit.SetNextTxRTimer() {
    uint32_t diff, nextTxR_local;
    uint32_t now_time = call TxRTimer.getNow();
    uint32_t now_global = now_time + node_time_offset;
    uint32_t origin_shift = ((my_depth%2) == 1) ? FRAME_PERIOD/2 : 0;
    if(!time_synced) {
      call Logger.logData(TXDEBUG, __LINE__); // No timer to be set when not synced
      return;
    }
    nextTxR_local =
      ((now_global-origin_shift)/FRAME_PERIOD + 1)*FRAME_PERIOD + origin_shift + GUARD_TIME - node_time_offset;
    diff = nextTxR_local - call TxRTimer.getNow();
    call TxRTimer.start(diff);
  } // End PIPTransmit.SetNextTxRTimer()

  /** Start the alarm to send the log. This is used for debugging
      only. When logging over radio doesnt work */
  async command void PIPTransmit.startSendLogAlarm() {
     
    if(call sendLogAlarm.isRunning()) {
      call sendLogAlarm.stop(); // To make this call idempotent
      call sendLogAlarm.start(DEFAULT_CHANNEL_TIMEOUT);
    } else {
      // The first time the log alarm is set for non-SINK nodes, it is
      // set more conservatively
      if(TOS_NODE_ID == SINK) { call sendLogAlarm.start(DEFAULT_CHANNEL_TIMEOUT*2); }
      else { call sendLogAlarm.start(DEFAULT_CHANNEL_TIMEOUT*2); }
    }
  }




  /*=============== End PIPTransmit commands ===============*/

  /**************** CaptureSFD events ****************/
  /**
   * The CaptureSFD event is actually an interrupt from the capture pin
   * which is connected to timing circuitry and timer modules.  This
   * type of interrupt allows us to see what time (being some relative value)
   * the event occurred, and lets us accurately timestamp our packets.  This
   * allows higher levels in our system to synchronize with other nodes.
   *
   * Because the SFD events can occur so quickly, and the interrupts go
   * in both directions, we set up the interrupt but check the SFD pin to
   * determine if that interrupt condition has already been met - meaning,
   * we should fall through and continue executing code where that interrupt
   * would have picked up and executed had our microcontroller been fast enough.
   */
  void CaptureSFD_captured_helper(uint16_t time);
  async event void CaptureSFD.captured(uint16_t time) {
    CaptureSFD_captured_helper(time);
  } // End CaptureSFD.captured()
  // XXXX: many changes here
  void CaptureSFD_captured_helper(uint16_t time) {
    uint32_t time32 = time16to32(time, call TxRTimer.getNow());
    nx_uint8_t *taddr;
    timesync_radio_t *timesync;

    if(tx_m_state == TX_S_SFD) {
      /* SB : Following seqno is  used for debugging purpose only.*/
      uint32_t seqno = *((nx_uint16_t*)m_msg->data);
 
      if(pip_state == PIP_S_RXRAD_HALF) { call Logger.logData(TXDEBUG,__LINE__); }
      call Logger.logData(RAD_TX_SFD_CAP, seqno);
      atomic tx_m_state = TX_S_EFD; // Next to expect is an EFD
      sfdHigh = TRUE;
      call CaptureSFD.captureFallingEdge();
      call PacketTimeStamp.set(m_msg, time32);

      // Random seed init
      if(!seed_init_done) {
	call SeedInit.init((uint16_t)time32);
	seed_init_done = TRUE;
      }

      // Should write global timestamp onto packet
      taddr = m_msg->data + (call PacketTimeSyncOffset.get(m_msg) - sizeof(cc2420_header_t));
      timesync = (timesync_radio_t*)taddr;
      *timesync  = time32 + node_time_offset; /* SB */
      call CSN.clr();
      call TXFIFO_RAM.write( call PacketTimeSyncOffset.get(m_msg), (uint8_t*)timesync, sizeof(timesync_radio_t) );
      call CSN.set();

      // Disabling ACK FIFOP
      call PIPReceive.InterruptFIFOP_disable();

      // Note: not releasing the chip's SPI bus lock.
      // Can begin SPI receive now
      if(rxfifo_state == FIFO_FULL) { call PIPReceive.tryReceiveSPI(); }
      else { prev_txrad_half_state.spi_work_done = TRUE; }

      if(call SFD.get()) {
	return; // SFD is still high, can return and wait for falling edge
      }
      /** Fall Through because the next interrupt was already received */
    } // End if(tx_m_state == TX_S_SFD)

    if(tx_m_state == TX_S_EFD) {

      if(pip_state == PIP_S_RXRAD_HALF) { call Logger.logData(TXDEBUG, __LINE__); }
      sfdHigh = FALSE;
      call CaptureSFD.captureRisingEdge();
      atomic tx_m_state = TX_S_ACK_WAIT;
      call Logger.logData(RAD_TX_EFD, CC2420_ACK_WAIT_DELAY);
      call TxRTimer.start(CC2420_ACK_WAIT_DELAY);
      if ( !call SFD.get() ) {
	return; // SFD is still low, can return and wait for rising edge
      }
      /** Fall Through because the next interrupt was already received */
    } // End if(tx_m_state == TX_S_EFD)

    if(tx_m_state == TX_S_ACK_WAIT) {
      if(pip_state == PIP_S_RXRAD_HALF) { call Logger.logData(TXDEBUG, __LINE__); }
      // Assuming this to be an ACK
      prev_txrad_half_state.ack_recd = TRUE;
      //prev_txrad_half_state.expecting_ack_fifop = TRUE;
      prev_txrad_half_state.expecting_ack_fifop = FALSE;
      txfifo_state = FIFO_EMPTY; // Current pkt in TXFIFO is no more useful
      call Logger.logData(RAD_ACK_RECD, 0);
      call TxRTimer.stop(); // Cancel the ack timer
      // Set timer will be done in checkAndSwitchToRx()
      //call PIPTransmit.SetNextTxRTimer(); // Set the timer for the next PIP_S_TXRAD_HALF phase
      atomic tx_m_state = TX_S_ACK_EFD_WAIT;
      signalSendDone(SUCCESS);
    }

    if ( !m_receiving ) {
      sfdHigh = TRUE;
      call CaptureSFD.captureFallingEdge();
      call CC2420Receive.sfd( time32 );
      m_receiving = TRUE;
      m_prev_sfd_capture_time = time;
      if ( call SFD.get() ) {
	// SFD is still high, can return and wait for falling edge
	return;
      }
    }

    sfdHigh = FALSE;
    call CaptureSFD.captureRisingEdge();
    m_receiving = FALSE;

    if ( time - m_prev_sfd_capture_time < 10 ) {
      call CC2420Receive.sfd_dropped();
      if (m_msg)
	call PacketTimeStamp.clear(m_msg);
    }

    if(tx_m_state == TX_S_ACK_EFD_WAIT) {
      atomic tx_m_state = TX_S_STARTED;

      prev_txrad_half_state.radio_work_done = TRUE;
      
      call PIPReceive.checkAndSwitchToRx();

      //signalSendDone(SUCCESS);
    }

  } // End CaptureSFD_captured_helper()
  /*=============== End CaptureSFD events ===============*/

  /**************** ChipSpiResource Events ****************/
  async event void ChipSpiResource.releasing() {
  } // End ChipSpiResource.releasing()
  /*=============== End ChipSpiResource Events ===============*/

  /**************** CC2420Receive Events ****************/
  /**
   * If the packet we just received was an ack that we were expecting,
   * our send is complete.
   */
  async event void CC2420Receive.receive( uint8_t type, message_t* ack_msg ) {
    if ( type == IEEE154_TYPE_ACK && m_msg) {
      // This would never happen in PIP since we are not using
      // software ACKs
      call Logger.logData(TXDEBUG, __LINE__);
    }
  } // End CC2420Receive.receive()
  /*=============== End CC2420Receive Events ===============*/

  /**************** SpiResource Events ****************/
  void SpiResource_granted_helper();
  event void SpiResource.granted() {
    atomic { SpiResource_granted_helper(); }
  } // End SpiResource.granted()
  void SpiResource_granted_helper() {
    uint8_t num_waiting_tasks = 0;
    holding_spi = TRUE;
    if(tx_m_state == TX_S_LOAD) num_waiting_tasks++;
    if(tx_m_state == TX_S_BEGIN_TRANSMIT) num_waiting_tasks++;
    if(rx_m_state == RX_S_FLUSH) num_waiting_tasks++;
    if(rx_m_state == RX_S_RX_LENGTH) num_waiting_tasks++;
    if(pip_state == PIP_S_ACQ_SPI) num_waiting_tasks++;
    if(num_waiting_tasks != 1) {
      call Logger.logData(TXDEBUG, __LINE__);
      call PIPTransmit.releaseSpiResource();
      return;
    }
    call Logger.logData(TXDEBUG, __LINE__);
    if(tx_m_state == TX_S_LOAD) { loadTXFIFO(); }
    else if(tx_m_state == TX_S_BEGIN_TRANSMIT) { attemptRadioSend(); }
    else { signal PIPTransmit.SpiResourceGranted(); }
  } // End SpiResource_granted_helper()
  /*=============== End SpiResource Events ===============*/
  
  /**************** TXFIFO Events ****************/
  /**
   * The TXFIFO is used to load packets into the transmit buffer on the
   * chip
   */
  void TXFIFO_writeDone_helper(uint8_t* tx_buf, uint8_t tx_len, error_t error);
  async event void TXFIFO.writeDone(uint8_t* tx_buf, uint8_t tx_len, error_t error) {
    atomic { TXFIFO_writeDone_helper(tx_buf, tx_len, error); }
  } // End TXFIFO.writeDone()
  void TXFIFO_writeDone_helper(uint8_t* tx_buf, uint8_t tx_len, error_t error) {

  
    call Logger.logData(SPI_TX_DONE, 0);   
    is_txspi_in_progress=FALSE;
    txfifo_state = FIFO_FULL;
    call CSN.set();
    call PIPTransmit.releaseSpiResource();
    atomic tx_m_state = TX_S_BEGIN_TRANSMIT;

    if(prev_rxrad_half_state.txspi_pkt_type == PIP_PKT_CONN_REQ || pip_state==PIP_S_ROUTING ) {
      // Should send right away

      if(call PIPTransmit.acquireSpiResource() == SUCCESS) { attemptRadioSend(); }
      /* else attemptRadioSend() will be called from SpiResource.granted() */
      return;
    }
    // For all nodes, the timer is set for the first time only after
    // the first (data) packet is filled into the TXFIFO.  This is to
    // avoid a situation where a timer fires in the middle of a TX SPI
    // operation.
    if(!first_pkt_sent) {
      first_pkt_sent = TRUE;
      call PIPTransmit.SetNextTxRTimer();
    }
  } // End TXFIFO_writeDone_helper()
  
  async event void TXFIFO.readDone( uint8_t* tx_buf, uint8_t tx_len, 
				    error_t error ) {
  }
  /*=============== End TXFIFO Events ===============*/   

  /**************** CC2420Config events ****************/
  event void CC2420Config.syncDone( error_t error ) {
    // Not used currently
  } // End CC2420Config.syncDone()
  /*=============== End CC2420Config events ===============*/
  
  /**************** Timer Events ****************/
  /**
   * This will happen under the following circumstances:
   * (1) Its right now PIP_S_RXRAD_HALF, but its time to change to the
   * next PIP_S_TXRAD_HALF phase
   * (2) Its right now PIP_S_TXRAD_HALF, and ack timeout has happened
   */
  void TxRTimer_fired_helper();
  async event void TxRTimer.fired() {
    atomic { TxRTimer_fired_helper(); }
  } // End TxRTimer.fired()
  // XXXX: some state checking
  void TxRTimer_fired_helper() {
    if(pip_state == PIP_S_RXCONN_REQ) { call PIPReceive.RxConnReqTimerFired(); return; }

    if(tx_m_state == TX_S_ACK_WAIT) { //Timeout for ack

      if(!prev_txrad_half_state.pkt_transmitted) {
	call Logger.logData(TXDEBUG, __LINE__);
      }
      prev_txrad_half_state.ack_recd = FALSE;
      txfifo_state = FIFO_EMPTY; // Will be refilled
      atomic tx_m_state = TX_S_STARTED;
      call Logger.logData(RAD_ACK_TIMEOUT, 0);
      PIPStats.num_ack_timeout++;
      prev_txrad_half_state.radio_work_done = TRUE;
      prev_txrad_half_state.expecting_ack_fifop = FALSE;
      signalSendDone(FAIL);
      call PIPReceive.checkAndSwitchToRx();
      return;
    } // End if(tx_m_state == TX_S_ACK_WAIT)

    // This timer fired in some state other than TX_S_ACK_WAIT
    if(pip_state == PIP_S_TXRAD_HALF) { call Logger.logData(TXDEBUG, __LINE__); }
    call Logger.logData(RAD_TX_TIMER, PIPStats.num_frame_periods++);
    call Logger.logData(Q_LEN, q_len);
    init_txrad_state(PIP_S_TXRAD_HALF); // This is when we switch state to PIP_S_TXRAD_HALF


    call Logger.logData(PIP_STATE_LOG, pip_state);
    if(txfifo_state == FIFO_FULL) {
      // There is something to send
     
      if(tx_m_state != TX_S_BEGIN_TRANSMIT) {
	call Logger.logData(TXDEBUG, __LINE__);
      }
      if(call PIPTransmit.acquireSpiResource() == SUCCESS) { attemptRadioSend(); }
      /* else attemptRadioSend() will be called from SpiResource.granted() */
      // End if(txfifo_state == FIFO_FULL)
    } else if(rxfifo_state == FIFO_FULL) {

      prev_txrad_half_state.radio_work_done = TRUE;
      call PIPReceive.tryReceiveSPI();
      if(!prev_txrad_half_state.rxspi_initiated) {
	prev_txrad_half_state.spi_work_done = TRUE;
	call PIPReceive.checkAndSwitchToRx();
      }
      // End else if(rxfifo_state == FIFO_FULL)
    } else {

      // Nothing to do in this PIP_S_TXRAD_HALF, switch to PIP_S_RXRAD_HALF
      prev_txrad_half_state.radio_work_done = prev_txrad_half_state.spi_work_done = TRUE;
      call PIPReceive.checkAndSwitchToRx();
    }

  } // End TxRTimer_fired_helper()


  /* SB: realease SPI bus, reset tx_m_state, stop the backoff timer &
     switch the channel, Note : after switching the channel node sends
     the data over serial. */
  void sendLogAlarm_fired_helper();
  async event void sendLogAlarm.fired() {
    atomic { sendLogAlarm_fired_helper(); }
  } // End sendLogAlarm.fired()
  void sendLogAlarm_fired_helper() {    
    bool radio_log_send;
    call Logger.logData(TXDEBUG, __LINE__);
    
      
    // Indication that alarm is fired
    call Leds.set(4);
    call BusyWait.wait(500000);
    call Leds.set(0);
    call BusyWait.wait(500000);
    call Leds.set(4);
    
    atomic tx_m_state = TX_S_STARTED;
    rx_m_state = RX_S_STARTED;
    call TxRTimer.stop();
    m_msg = NULL;
    radio_log_send = RADIO_LOG_SEND;
    
    if(radio_log_send) { 
    
    } else { // Send over SPI
      call SpiResource.release();
      call ChipSpiResource.attemptRelease();
      pip_state = PIP_S_PRINT_LOG;
      call Logger.logData(PIP_STATE_LOG, pip_state);
      call Logger.dumpLog(FALSE);
    }
  } // End sendLogAlarm_fired_helper()
  
     
    


  /*=============== End Timer Events ===============*/
  /*=========Routing events=======*/
  async event void Routing.routingDone(){}
  /*=========ENDRouting events=======*/
  
} // End implementation
