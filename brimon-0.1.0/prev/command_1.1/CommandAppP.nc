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
#include "Command.h"

/**
 * the implementation code that sends to and receives from the application running on system
 */

module CommandAppP {
	uses {
		interface Timer<TMilli>;
		interface SplitControl as Control;
		interface SplitControl as AMControl;
		interface Leds;
		interface Boot;
		interface Receive;
		interface AMSend;
		interface Packet;
		interface Command;
		interface Routing;
		interface TimeSync;
		interface Sense;
		interface Collect;
		//interface Transfer;
		interface LogRead as SensorDataRead;
	}
}

implementation {

	message_t packet;

	bool locked = FALSE;
	uint32_t counter = 0;

	void transfer();
	void transfer_actual();

	/** execution on mote booting */
	event void Boot.booted() {
		call Control.start();
		call AMControl.start();
	}

	/** execution on AMControl up */
	event void AMControl.startDone(error_t err) {
		if (err != SUCCESS) {
			call AMControl.start();
		}
	}

	/** execution on AMControl down */
	event void AMControl.stopDone(error_t err) {}

	/** executed when the coompletion of a command execution is signalled by the component responsible for the execution.
	 * For completion of routing, the routing tree is sent in the payload to the system
	 */
	event void Command.execCommandDone(error_t error, uint16_t commType) {
		int i;
		if (locked) {
			return;
		}
		else {
			test_serial_msg_t* rcm = (test_serial_msg_t*)call Packet.getPayload(&packet, sizeof(test_serial_msg_t));
			if (rcm == NULL) {return;}
			if (call Packet.maxPayloadLength() < sizeof(test_serial_msg_t)) {return;}
			//additional payload handling for routing completion packet
			if(commType==ROUTING) {
				rcm->counter = call Routing.getNumNodes();
				for(i=0;i<call Routing.getNumNodes();i++) {
					rcm->payload[i] = call Routing.getParent(i);
				}
			}
			rcm->type = commType + DIFF_COMMAND_COMPLETE;
			if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(test_serial_msg_t)) == SUCCESS) {
				locked = TRUE;
			}
		}
	}

	/** called when we need to transfer sensed data log to system */
	void transfer() {
		call SensorDataRead.seek(0);
		counter=0;
	}

	/** underlying function called when the seek called by transfer() finishes execution */
	void transfer_actual() {
		if (locked) {
			call Timer.startOneShot(10);
		}
		else {
			test_serial_msg_t* rcm = (test_serial_msg_t*)call Packet.getPayload(&packet, sizeof(test_serial_msg_t));
			if (rcm == NULL) {call Timer.startOneShot(10); return;}
			if (call Packet.maxPayloadLength() < sizeof(test_serial_msg_t)) {call Timer.startOneShot(10); return;}
			if(call SensorDataRead.read(rcm->payload, sizeof(rcm->payload)) !=SUCCESS)
				call Timer.startOneShot(10);
			else {
				rcm->type = TRANSFER;
				rcm->counter = counter;
				if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(test_serial_msg_t)) == SUCCESS) {
					locked = TRUE;
					counter++;
				}
				call Timer.startOneShot(10);
			}
		}
	}

	/** Executed when a previously set timer fires.
	 * Checks if the log has been completely transferred.
	 * If yes, then it signals successfully finished transfer operation.
	 * Else, it continues with the next execution of transfer_actual().
	 */
	event void Timer.fired() {
		if(counter*sizeof(30) < (call Routing.getNumNodes()*call Sense.getLogSize()))
			transfer_actual();
		else
			signal Command.execCommandDone(SUCCESS, TRANSFER);
	}

	/** Executed when the mote receives a message from the Java application running on the system.
	 * Carries a command to be executed on the mote.
	 * If the command is transfer, call the transfer() function.
	 * Else, pass the command to commaand execution subsystem provided by CommandC in the form of Command interface.
	 */
	event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len) {
		if (len != sizeof(test_serial_msg_t))
			return bufPtr;
		else {
			test_serial_msg_t* rcm = (test_serial_msg_t*)payload;
			if(rcm->type==TRANSFER) {
				atomic transfer();
			}
			else {
				call Command.execCommand(rcm->type);
			}
			//call Leds.set(rcm->payload);
			return bufPtr;
		}
	}

	/** executed when a packet has been sent */
	event void AMSend.sendDone(message_t* bufPtr, error_t error) {
		if (&packet == bufPtr) {
			locked = FALSE;
		}
	}

	event void Control.startDone(error_t err) {}

	event void Control.stopDone(error_t err) {}

	event void Routing.routeDone(error_t error) {}

	event void TimeSync.startDone(error_t error) {}

	event void TimeSync.stopDone(error_t error) {}

	event void Sense.senseDone(error_t error) {}

	event void Collect.collectDone(error_t error) {}

/*
	event void Transfer.transferDone(error_t error) {}
*/

	event void SensorDataRead.readDone(void* buf, storage_len_t len, error_t error) {}

	event void SensorDataRead.seekDone(error_t error) {
		transfer_actual();
	}
}
