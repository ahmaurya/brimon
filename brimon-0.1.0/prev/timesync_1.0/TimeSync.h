#ifndef SEND_RECV_H
#define SEND_RECV_H

#define NODE_ID 2
#define NODES 3
#define TWK 6400L
#define TSL 960000L
#define SYNC_PERIOD 20

# include <printf.h>
# include <Timer.h>
# include <CC2420TimeSyncMessage.h>

enum {
  	AM_COUNT_MSG = 1,
};

typedef nx_struct timesync_msg {
	nx_uint16_t count;
	nx_uint8_t src;
	nx_uint32_t offset;
	nx_uint32_t sendtime;
}timesync_msg_t;

#endif
