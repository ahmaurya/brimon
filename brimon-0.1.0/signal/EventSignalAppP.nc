/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "EventSignal.h"

module EventSignalAppP @safe() {
	uses {
		interface Boot;
		interface Leds;
		interface CC2420Config;
		interface AMSend;
		interface Timer<TMilli> as MilliTimer;
		interface SplitControl as AMControl;
		interface Packet;
	}
}

implementation {

	message_t packet;
	radio_beacon_msg_t* rcm = NULL;

	event void Boot.booted() {
		call AMControl.start();
	}

	task void prepare() {
		call CC2420Config.setChannel(BEACON_CHANNEL);
		call CC2420Config.sync();
		while(rcm == NULL)
			rcm = (radio_beacon_msg_t*)call Packet.getPayload(&packet, sizeof(radio_beacon_msg_t));
		rcm->type = BEACON_MSG;
	}

	event void AMControl.startDone(error_t err) {
		if (err == SUCCESS) {
			post prepare();
			call MilliTimer.startOneShot(1000);
		}
		else {
			call AMControl.start();
		}
	}

	event void AMControl.stopDone(error_t err) {}

	event void CC2420Config.syncDone(error_t error) {}

	event void MilliTimer.fired() {
		call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_beacon_msg_t));
    }

	event void AMSend.sendDone(message_t* bufPtr, error_t error) {
		call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_beacon_msg_t));
    }
}
