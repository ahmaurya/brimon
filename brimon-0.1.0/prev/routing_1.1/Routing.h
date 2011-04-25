#ifndef SEND_RECV_H
#define SEND_RECV_H

#define NODES 2
#define TWK 6400L
#define TSL 960000L
#define SYNC_PERIOD 20
#define GOOD_LINK -85
#define BAD_LINK -100
#define QUEUE_LENGTH 50
#define GIVE_UP 10

#ifndef PRINTF_H
#define PRINTF_H
#include <printf.h>
#endif

#include <Timer.h>
#include <CC2420TimeSyncMessage.h>

enum {
  	AM_ROUTING_MSG = 1,
};

typedef nx_struct routing_tree{
	nx_int8_t parent;
	nx_bool good_bad;
};

typedef nx_struct routing_msg {
	nx_uint8_t type;
	nx_uint8_t src;
	nx_uint8_t dest;
}routing_msg_t;


typedef nx_struct tree_msg {
	nx_uint8_t type;
	nx_uint8_t src;
	nx_uint8_t dest;
	nx_int8_t tree[NODES];
}tree_msg_t;


typedef nx_struct link_msg {
	nx_uint8_t type;
	nx_uint8_t src;
	nx_uint8_t dest;
	nx_int8_t link_rssi[NODES];
	nx_int8_t flag;
}link_msg_t;

#endif
