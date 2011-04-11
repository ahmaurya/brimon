#ifndef SEND_RECV_H
#define SEND_RECV_H

enum {
  	AM_COUNT_MSG = 1,
};

typedef nx_struct brimon_msg {
	nx_uint16_t type;
	nx_uint16_t src;
	//nx_uint8_t dst;
	nx_uint32_t count;
	nx_uint32_t sendtime;
} brimon_msg_t;

#endif
