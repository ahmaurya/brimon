#ifndef TIMESYNC_H
#define TIMESYNC_H

#define NUM_NODES 6
#define TWK 6400L
#define TSL 96000L
#define SYNC_PERIOD 20

# include <printf.h>
# include <Timer.h>
# include <CC2420TimeSyncMessage.h>

typedef nx_struct timesync_msg {
	nx_uint16_t count;
	nx_uint8_t src;
	nx_uint32_t offset;
	nx_uint32_t sendtime;
} timesync_msg_t;

#endif
