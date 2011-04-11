#include "Timer.h"
#include "DummyTwo.h"

module DummyTwoC {
  uses {
    interface Boot;
    interface Leds;
    interface Timer<TMilli> as MilliTimer;
    interface AMSend;
    interface Packet;
    interface Receive;
    interface SplitControl as AMControl;
  }
}
implementation {

  message_t packet;

  bool locked;
  uint16_t counter = 0;

  event void Boot.booted() {
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      call MilliTimer.startPeriodic(1000);
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
    // to do
  }

  event void MilliTimer.fired() {
    counter++;
    if (locked)
      return;
    else {
      radio_count_msg_t* rcm = (radio_count_msg_t*)call Packet.getPayload(&packet, sizeof(radio_count_msg_t));
      if (rcm == NULL)
			return;
      rcm->counter = counter;
      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_count_msg_t)) == SUCCESS)
			locked = TRUE;
    }
  }

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (error == SUCCESS && &packet == bufPtr) {
      locked = FALSE;
    }
  }

  event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len) {
    if (len != sizeof(radio_count_msg_t)) {return bufPtr;}
    else {
      radio_count_msg_t* rcm = (radio_count_msg_t*)payload;
      call Leds.set(rcm->counter);
      return bufPtr;
    }
  }

}
