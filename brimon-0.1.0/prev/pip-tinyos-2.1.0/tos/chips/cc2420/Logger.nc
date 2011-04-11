interface Logger {
  async command error_t init();
  async command error_t logData(uint8_t type,uint32_t value);
  async command error_t dumpLog(bool cald_from_trans_module);   
  async command error_t fillNextLogEntry(message_t*, uint32_t entry_no );
  async command void  clearExptalLog();
  async command void  clearLoggingLog();
  async command void reset();
}
