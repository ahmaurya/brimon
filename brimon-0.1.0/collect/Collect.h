/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Jeet Patani
 * Copyright (C) 2011 Jeet Patani
 */

#ifndef COLLECT_H
#define COLLECT_H

#define NODES 3			// Number of nodes in the topology
#define QUEUE_LENGTH 20		// Packet queue length
#define DATA_LEN 50		//
#define TOTAL_PKT 10		// Total Packets per block
#define TOTAL_BLOCKS 40		// Total Blocks
#define PKT_SIZE 20 		// Data size per packet
#define BLK_SIZE 200		// Size of Block
#define GIVE_UP 10		// Number of retransmissions before giving up
#define INTER_PKT_TIME 1	// Time in ms at which next packet from queue is to be sent.
#define INTER_LOG_TIME 2	// Time in ms at which next log operation is to be performed.
#define BUFFER_SIZE 50 		// Size of buffer used for LogRead and LogWrite operations
#define TIME_OUT_REQ 5000	// Request timeout valuse in ms.
#define GIVE_UP_TRANSFER 100000	// Giveup Data Transfer after GIVE_UP_TRANSFER time.

#define DATA_REQUEST_MSG 0 	// Type of data request message. ask_data_msg_t packet.
#define TRANSFER_DONE 3		// Type of Transfer Done packet
#define ACK_REQUEST_MSG 1	// Type of ack request message. ask_data_msg_t packet.
#define ACK_REPLY_MSG 0 	// Type of ack reply messafe. ack_msg_t packet
#define BLK_SUCCESS 2		// Type of packet indicating successful transmission of block.ask_data_msg_t packet.
#define DATA_MSG_TYPE 0		// Type of data message. data_msg_t pacekt.


#ifndef PRINTF_H
#define PRINTF_H
# include <printf.h>
#endif

# include <Timer.h>
# include <CC2420TimeSyncMessage.h>
# include "StorageVolumes.h"

enum {
  	AM_COLLECT_MSG = 40,
};

/*
	ask_data_msg_t will be generated by Head Node. It will contain type, source, destination and block field.
	Destination will respond with data transfer of specified block.
*/

typedef nx_struct ask_data_msg {
	nx_uint8_t type;
	nx_uint8_t block;
	nx_uint8_t src;
	nx_uint8_t dest;
}ask_data_msg_t;

/*
	data_msg_t is used for data transfer. It will contain type, block id, packet sequence number, source id, destination id and data.
*/

typedef nx_struct data_msg {
	nx_uint8_t type;
	nx_uint8_t block;
	nx_uint8_t seq;
	nx_uint8_t src;
	nx_uint8_t dest;
	nx_uint8_t data[PKT_SIZE];
}data_msg_t;

/*
	ack_msg_t is used to send NACK. It will be sent at the end of transmission of each block containing information about which
	packets received and which are not.
	It will have type, block id, source id, destination id and an array containing NACK information.
*/

typedef nx_struct ack_msg {
	nx_uint8_t type;
	nx_uint8_t block;
	nx_uint8_t src;
	nx_uint8_t dest;
	nx_uint8_t ack[TOTAL_PKT];
}ack_msg_t;


#endif
