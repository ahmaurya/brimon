#ifndef TIMESYNC_H
#define TIMESYNC_H

#define TWK 6400L		// Wake up Interval in 32KHz
#define TSL 96000L		// Sleep Interval in 32KHz
#define SYNC_PERIOD 50		// Time gap between packets in milliseconds
#define STOP_DUTY_CYCLE 5	// Number of packets to be sent to stop duty cycle.
#define START_CYCLE 0		// Type of packet to indicate start duty cycle
#define STOP_CYCLE 1		// Type of packet to indicate stop duty cycle


#ifndef PRINTF_H
#define PRINTF_H
# include <printf.h>
#endif

# include <Timer.h>
# include <CC2420TimeSyncMessage.h>

enum {
  	AM_TIMESYNC_MSG = 130,
};

/*
	timesync_msg_t is used for timesync messages.
	It contains type, source, offset and sendtime.

*/
typedef nx_struct timesync_msg {
	nx_uint8_t type;
	nx_uint8_t src;
	nx_uint32_t offset;
	nx_uint32_t sendtime;
} timesync_msg_t;

#endif
