/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Jeet Patani
 * Copyright (C) 2011 Jeet Patani
 */

#ifndef ROUTING_H
#define ROUTING_H

#define NODES 2 		// Number of Nodes in Topology
#define GOOD_LINK -85		// RSSI Value for good quality link
#define BAD_LINK -100		// RSSI Value for bad quality lonk
#define QUEUE_LENGTH 20		// Packet Queue length
#define GIVE_UP 10		// Number of attempts no be made before giving up sending packet
#define MAX_HELLO_TIME	2000	// Maximum possible time for exchanging hello messages. It will anyhow start routing after this time. It is in milliseconds.
#define HELLO_MSG_INTERVAL 20	// Time in ms between two hello messages
#define NUM_HELLO 10		// Number of Hello Messages
#define HELLO_MESSAGE_TYPE 0	// Type of Hello Message. (3 Bytes)
#define INTR_PKT_INTERVAL 1	// Gap between transmitting two packets in ms
#define GIVE_UP_TIME 20000	// Give up after GIVE_UP_TIME ms
#define ASK_FOR_LINKS 0 	// Type of tree message asking for link state ( 3 + NODES Bytes)
#define ROUTING_DONE_MSG 1	// Type of tree message indicating routing done ( 3 + NODES Bytes)
#define LINK_MSG_TYPE 0 	// Type of link message ( 4 + NODE Bytes )
#define PERIODIC_TIMER	5000	// Check at every PERIODIC_TIMER milliseconds whether the routing has been done or not.

#ifndef PRINTF_H
#define PRINTF_H
# include <printf.h>
#endif

#ifndef PRINTF_H
#define PRINTF_H
# include <Timer.h>
#endif

# include <CC2420TimeSyncMessage.h>

enum {
  	AM_ROUTING_MSG = 60,
};

// routing_msg_t defines packet structure of Hello Message. It is used only to fine RSSI value.

typedef nx_struct routing_msg {
	nx_uint8_t type;
	nx_uint8_t src;
	nx_uint8_t dest;
}routing_msg_t;

// tree_msg_t is used to convey tree information. It is used to ask for link state table as well as to inform final Routing Tree.
// Different types are used for asking for link state and sending final tree.

typedef nx_struct tree_msg {
	nx_uint8_t type;
	nx_uint8_t src;
	nx_uint8_t dest;
	nx_int8_t tree[NODES];
}tree_msg_t;

// link_msg_t is used to convey link state information of neighbors of a node.

typedef nx_struct link_msg {
	nx_uint8_t type;
	nx_uint8_t src;
	nx_uint8_t dest;
	nx_int8_t link_rssi[NODES];
	nx_int8_t flag;
}link_msg_t;


#endif
