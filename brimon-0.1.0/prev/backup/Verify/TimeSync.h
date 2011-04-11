#ifndef SEND_RECV_H
#define SEND_RECV_H

enum {
  	AM_COUNT_MSG = 1,
};

typedef nx_struct timesync_msg {
	nx_uint16_t count;
	nx_uint16_t src;
	nx_uint32_t prev_sendtime;
	nx_uint32_t sendtime;
}timesync_msg_t;

#endif
