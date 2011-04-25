/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "Timer.h"
#include "Serial.h"

module SerialAppP {
  uses {
    interface SplitControl as Control;
    interface Leds;
    interface Boot;
    interface Receive;
    interface AMSend;
    interface Timer<TMilli> as MilliTimer;
    interface Packet;
  }
}
implementation {

  message_t packet;

  bool locked = FALSE;
  uint16_t counter = 0;

  event void Boot.booted() {
    call Control.start();
  }

  event void MilliTimer.fired() {
    counter++;
    if (locked) {
      return;
    }
    else {
      test_serial_msg_t* rcm = (test_serial_msg_t*)call Packet.getPayload(&packet, sizeof(test_serial_msg_t));
      if (rcm == NULL) {return;}
      if (call Packet.maxPayloadLength() < sizeof(test_serial_msg_t)) {
	return;
      }

      rcm->counter = counter;
      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(test_serial_msg_t)) == SUCCESS) {
	locked = TRUE;
      }
    }
  }

  event message_t* Receive.receive(message_t* bufPtr,
				   void* payload, uint8_t len) {
    if (len != sizeof(test_serial_msg_t)) {return bufPtr;}
    else {
      test_serial_msg_t* rcm = (test_serial_msg_t*)payload;
      if (rcm->counter & 0x1) {
	call Leds.led0On();
      }
      else {
	call Leds.led0Off();
      }
      if (rcm->counter & 0x2) {
	call Leds.led1On();
      }
      else {
	call Leds.led1Off();
      }
      if (rcm->counter & 0x4) {
	call Leds.led2On();
      }
      else {
	call Leds.led2Off();
      }
      return bufPtr;
    }
  }

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (&packet == bufPtr) {
      locked = FALSE;
    }
  }

  event void Control.startDone(error_t err) {
    if (err == SUCCESS) {
      call MilliTimer.startPeriodic(1000);
    }
  }
  event void Control.stopDone(error_t err) {}
}




