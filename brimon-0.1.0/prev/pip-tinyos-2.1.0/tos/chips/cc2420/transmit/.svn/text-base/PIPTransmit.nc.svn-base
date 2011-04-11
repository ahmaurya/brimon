interface PIPTransmit {
  async command void reset();
  
  async command void beginSPITransmit();
  async command void SetNextTxRTimer();
  async command void TxRTimer_start(uint32_t);
  async command void TxRTimer_stop();
  async command bool TxRTimer_isRunning();
  
  async command void startSendLogAlarm(); 

  async command error_t acquireSpiResource();
  async command error_t releaseSpiResource();
  async event void SpiResourceGranted();
  async command void setChannel(uint8_t); // Used for setting recv chnl also
  async command void signalSendDone(error_t);
  async command void signalStartDone();

  async command void signalIncomingConnReq(message_t *p_msg, uint8_t len);
  async command void signalIncomingData(message_t *p_msg, uint8_t len);
  async command void signalIncomingEOF(message_t *p_msg, uint8_t len);
  async command void signalIncomingSNACK(message_t *p_msg, uint8_t len);
  async command void signalIncomingTeardown(message_t *p_msg, uint8_t len);

  async command void signalIncomingHelloMsg(message_t *p_msg, uint8_t len);
  async command void signalIncomingLSQueryMsg(message_t *p_msg, uint8_t len);
  async command void signalIncomingLSResponseMsg(message_t *p_msg, uint8_t len);
  async command void signalIncomingRtgResetMsg(message_t *p_msg, uint8_t len);

}
