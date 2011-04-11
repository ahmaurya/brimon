interface PIPMac {
  /** This signals that PIP is ready for a connection */
  async event void startDone();
  async event void resetDone();
  
  /**** Reset PIPMAC ****/
  async command void reset(pip_state_t local_pip_state);

  /** Sending various kinds of PIP packets */
  async command error_t sendConnRequest(message_t *msg, uint8_t length);
  async command error_t sendData(message_t *msg, uint8_t length);
  async command error_t sendEOF(message_t *msg, uint8_t length);
  async command error_t sendSNACK(message_t *msg, uint8_t length);
  async command error_t sendTeardown(message_t *msg, uint8_t length);

  /** Sendiong various kinds of routing packets***/
  async command error_t sendHelloMsg(message_t *msg, uint8_t length);
  async command error_t sendLSQueryMsg(message_t *msg, uint8_t length);
  async command error_t sendLSResponseMsg(message_t *msg, uint8_t length);
  async command error_t sendRtgResetMsg(message_t *msg, uint8_t length);

  /** Signals that a certain kind of pkt has been sent */
  async event void sendConnRequestDone(message_t *msg, error_t status);
  async event void sendDataDone(message_t *msg, error_t status);
  async event void sendEOFDone(message_t *msg, error_t status);
  async event void sendSNACKDone(message_t *msg, error_t status);
  async event void sendTeardownDone(message_t *msg, error_t status);

  /** Similar signals for routing msgs**/
  async event void sendHelloMsgDone(message_t *bufPtr, error_t status);
  async event void sendLSQueryMsgDone(message_t *bufPtr, error_t status);
  async event void sendLSResponseMsgDone(message_t *bufPtr, error_t status);
  async event void sendRtgResetMsgDone(message_t *bufPtr, error_t status);
 


  /** Signal that a certain kind of pkt has been received */
  async event void incomingConnRequest(message_t *msg, uint8_t length);
  async event void incomingData(message_t *msg, uint8_t length);
  async event void incomingEOF(message_t *msg, uint8_t length);
  async event void incomingSNACK(message_t *msg, uint8_t length);
  async event void incomingTeardown(message_t *msg, uint8_t length);

  /**** Similar signals for routing pkts***/
  async event void incomingHelloMsg(message_t *msg, uint8_t length);
  async event void incomingLSQueryMsg(message_t *msg, uint8_t length);
  async event void incomingLSResponseMsg(message_t *msg, uint8_t length);  
  async event void incomingRtgResetMsg(message_t *msg, uint8_t length); 

}
