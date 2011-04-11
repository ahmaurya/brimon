interface PIPReceive {
  async command void tryReceiveSPI();
  async command void checkAndSwitchToRx();
  async command void switchToRxRadioMode();
  async command void RxConnReqTimerFired();
  async command void InterruptFIFOP_disable();
  async command void connectionDone();
  async command void resetFIFOP();
  async command void checkFIFOStatus();
  async command void reset();
}
