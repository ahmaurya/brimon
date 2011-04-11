#define NODE_ID 4
#define NODES 6
#ifndef SEND_RECV_H
#define SEND_RECV_H

# include <printf.h>
# include <Timer.h>
# include <CC2420TimeSyncMessage.h>

enum {
  	AM_COUNT_MSG = 1,
};

typedef nx_struct timesync_msg {
	nx_uint16_t count;
	nx_uint8_t src;
	nx_uint32_t prev_sendtime;
	nx_uint32_t sendtime;
	nx_uint32_t abc[2];
}timesync_msg_t;

#endif
