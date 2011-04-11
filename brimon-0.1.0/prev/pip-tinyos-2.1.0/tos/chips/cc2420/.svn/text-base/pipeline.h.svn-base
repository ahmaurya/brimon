#ifndef PIPELINE_H
#define PIPELINE_H

#include "message.h"
#include "CC2420TimeSyncMessage.h"
#include "Routing.h"

#define TXR_HALF_CYCLE 200
#define RXR_HALF_CYCLE TXR_HALF_CYCLE
#define GUARD_TIME 15
#define FRAME_PERIOD (TXR_HALF_CYCLE+GUARD_TIME+RXR_HALF_CYCLE+GUARD_TIME)

#define PIP_DEFAULT_CHANNEL 11

//#define PIP_RX_CHANNEL(nodeid) (11+(nodeid))

/** Circular queue implementation */
#define CIRC_QUEUE_MAX_ELEMENTS 10
#define CIRC_QUEUE_ARRAY_SIZE (CIRC_QUEUE_MAX_ELEMENTS+1)
norace nx_uint8_t front, rear, q_len;
norace message_t pipe_packet[CIRC_QUEUE_ARRAY_SIZE];

// Simulated error rate (percentage)
#define SIM_ERR_PERC 0
// Note the casting into uint32_t seems necessary for the
// multiplication to work; we finally have a 16-bit value though
#define SIM_ERR_FRAC_UINT16 (0x0000ffffU*(uint32_t)SIM_ERR_PERC/100)
// Errors are simulated by sending into a blackhole PAN address
#define BLACKHOLE_PAN 111
norace nx_uint8_t seed_init_done;

// Flow control enabled or no?
#define FLOW_CONTROL_ENABLED 0
// Early teardown (right after first set of data packets)
#define EARLY_TEARDOWN 0

// Number of packets is a multiple of 8, for ease of ack maintenance
#define  NUM_PACKS (32<<3)

/* SRC is changed as told by PCTool. This is then embedded in conn_req
   packet. A node after extracting this from the conn_req knows
   whether its a source or forwarder and sets im_source accordingly**/
uint8_t SRC=4; // This is now used by SINK only
bool  im_source=FALSE; // ALL nodes use this
/**** Following flag indicates whether current transfer is exptal or
      log transfer ***/
bool expt_or_log=TRUE;
#define SINK 1  // SINK id is always 1

uint8_t pip_nextNode=BLACKHOLE_PAN;  // next towards Sink
uint8_t pip_prevNode=BLACKHOLE_PAN;
/* Channels are assigned with the nodes & not links***/
uint8_t pip_nextNodeRxChannel=PIP_DEFAULT_CHANNEL; //next towards Sink
uint8_t pip_prevNodeRxChannel=PIP_DEFAULT_CHANNEL;
uint8_t pip_myRxChannel=PIP_DEFAULT_CHANNEL;
uint8_t my_depth=0;


/* following 2 are used during the routing phase only. Can't use
   txrad_half_state or rxrad_half_state as there is no pipelining
   during routing. These can be considered as non-blocking mutex for common spi
   resource in case of no_pipelining ***/

bool is_rxspi_in_progress=FALSE;
bool is_txspi_in_progress=FALSE;



// SB: ALL DEBUG MODULE VARS HERE
/** Following is actually time after which log is sent over serial(if
    enabled by RADIO_LOG_SEND flag) **/
#define DEFAULT_CHANNEL_TIMEOUT 2000000
/* This flag specifies whether to send the data over radio or serial
   1) 0 for serial 2) 1 for radio */
#define RADIO_LOG_SEND 0


enum {
#include "log-entries.h"
};
#define MAX_LOG_ENTRIES 128
typedef struct logging {
  uint32_t time;
  uint32_t value;
  uint8_t type;
} Log;
Log exptalLogArray[MAX_LOG_ENTRIES];

/** logging while transferring log. Two different array are used now
    for logging during exptn & during log collection.**/
Log loggingLogArray[MAX_LOG_ENTRIES];


/** Following var is set =TRUE when sendlogalarm is fired && remains
    true till  next resetstate **/
bool sending_log = FALSE;
#define NUM_STAT_ENTRIES STAT_END
Log statExptalLog[NUM_STAT_ENTRIES];
Log statLoggingLog[NUM_STAT_ENTRIES];

/**** SB: in_tranfer_phase is used by LoggerM (for logging more
      events during SNACK & EOF transmissions (commented now as bug
      is removed)  ***/ 
bool in_transfer_phase=TRUE;

typedef struct stats {
  // the total number of pkts sent (including retx, all pkt types)
  uint16_t num_pkts_sent;
  // the number of pkts recd w/o CRC error (all pkt types)
  uint16_t num_pkts_recd;
  // number of pkts recd w/ CRC error (all pkt types)
  uint16_t num_crc_err;
  // number of pkts sent w/ simulated CRC error (all pkt types)
  uint16_t num_sim_err;
  // number of times I timed out waiting for ACK
  uint16_t num_ack_timeout;
  // number of pkts for which max retries exceeded, excluding ConnReq
  // and TearDown
  uint16_t num_max_retry_exceeded;

  // the number of data pkts recd
  uint16_t num_data_pkts_recd;

  // the number of eof pkts recd/sent (excluding retx)
  uint16_t num_eof_pkts_recd, num_eof_pkts_sent;
  // the number of snack pkts recd/sent (excluding retx)
  uint16_t num_snack_pkts_recd, num_snack_pkts_sent;

  // total number of frame periods
  uint16_t num_frame_periods;
  // number of frame periods during which pkt was dropped due to queue being full
  uint16_t num_queue_full;
  // number of frame periods during which no pkt was sent due to queue being empty
  uint16_t num_queue_empty;
} stats_t;
stats_t PIPStats;

/** How much is this node behind the data source */
norace nx_uint32_t node_time_offset;
uint8_t curr_pip_channel = 0; // The current radio channel

bool first_pkt_sent = FALSE;
bool first_pkt_recd = FALSE;
/** flag to indicate whether node is sync state or not**/
bool time_synced = FALSE;



typedef enum {
  RX_S_STOPPED,
  RX_S_STARTED,
  RX_S_RX_LENGTH,
  RX_S_RX_FCF,
  RX_S_RX_PAYLOAD,
  RX_S_FLUSH,
} cc2420_receive_state_t;
/** The current state of the receive module.  Takes one of the values
    from enum cc2420_receive_state_t. */
cc2420_receive_state_t rx_m_state = RX_S_STOPPED;

typedef enum {
  TX_S_STOPPED,
  TX_S_STARTED,
  TX_S_LOAD,
  TX_S_SAMPLE_CCA,
  TX_S_BEGIN_TRANSMIT,
  TX_S_SFD,
  TX_S_EFD,
  TX_S_ACK_WAIT,
  TX_S_ACK_EFD_WAIT,
  TX_S_CANCEL,
} cc2420_transmit_state_t;
/** The current state of the transmit module.  Takes one of the values
    from enum cc2420_transmit_state_t. */
cc2420_transmit_state_t tx_m_state = TX_S_STOPPED;

/** PIP states as per the protocol**/
typedef enum {
  PIP_S_CONFIG=11,
  PIP_S_ACQ_SPI=22,
  PIP_S_DEFAULT=33,
  PIP_S_RXCONN_REQ=44,
  PIP_S_TXCONN_REQ=55,
  PIP_S_RXRAD_HALF=66,
  PIP_S_TXRAD_HALF=77,
  PIP_S_ROUTING=88,
  PIP_S_PRINT_LOG=111,
} pip_state_t;
pip_state_t pip_state;

typedef nx_struct ConnPktPayload {
  nxle_uint8_t connreq_or_logquery; //88 for connreq & 99 for logquery
  nxle_uint8_t noHopsToGo; 
  nxle_uint8_t path[MAX_HOPS];  // Sink is implicit (not mentioned in path)
  nxle_uint8_t channelInfo[MAX_HOPS+1]; // As many channels as number
					// of nodes (Sink included)
  nxle_uint8_t totalHopsInPath;
} ConnPktPayload_t; 

#define CONN_REQ_REPEAT 3
/* Gap between retxs of connection request: without this, the receiver
   is slower than the sender.  Unit: ticks in micro-sec. */
#define CONN_REQ_GAP (30*30)
#define TEAR_DOWN_REPEAT 3
uint8_t num_conn_req_sent = 0;
uint8_t num_conn_req_recd = 0;
uint8_t num_tear_down_sent = 0;
uint8_t num_tear_down_recd = 0;

#define FIFO_EMPTY 0
#define FIFO_FULL 1

uint8_t txfifo_state, rxfifo_state;
bool holding_spi = FALSE;


/* Encapsulate the state corresponding to the activities in a
   TXRAD_HALF */
typedef struct txrad_half_state {
  bool pkt_transmitted; // was a pkt transmitted?
  bool ack_recd; // was an ack recd? (valid only if pkt_transmitted is TRUE)
  bool expecting_ack_fifop; // are we expecting an ack fifop?
  bool rxspi_initiated; // was an rxspi done?
  bool rxspi_crc_error; // did the recd pkt have a CRC error?
  bool radio_work_done; // is the radio related work done?
  bool spi_work_done; // is the spi related work done?
  pip_pkt_type_t txrad_pkt_type; // what was the type of pkt sent over radio?
} txrad_half_state_t;
/** What happened in the previous TXRAD_HALF phase?  This is
    guaranteed to have valid entries only in the RXRAD_HALF phase. */
txrad_half_state_t prev_txrad_half_state;

/* Encapsulate the state corresponding to the activities in a
   RXRAD_HALF */
typedef struct rxrad_half_state {
  bool pkt_received; // was a pkt received?
  bool txspi_initiated; // was a txspi done?
  pip_pkt_type_t txspi_pkt_type; // what was the type of the pkt sent to TXFIFO?
} rxrad_half_state_t;
/** What happened in the previous RXRAD_HALF phase?  This is
    guaranteed to have valid entries only in the TXRAD_HALF phase. */
rxrad_half_state_t prev_rxrad_half_state;

inline void init_txrad_state(pip_state_t ps) {
  pip_state = ps;
  prev_txrad_half_state.pkt_transmitted = prev_txrad_half_state.ack_recd =
    prev_txrad_half_state.expecting_ack_fifop =
    prev_txrad_half_state.rxspi_initiated = prev_txrad_half_state.rxspi_crc_error =
    prev_txrad_half_state.radio_work_done = prev_txrad_half_state.spi_work_done = FALSE;
  prev_txrad_half_state.txrad_pkt_type = 255;
}

inline void init_rxrad_state(pip_state_t ps) {
  pip_state = ps;
  prev_rxrad_half_state.pkt_received = prev_rxrad_half_state.txspi_initiated = FALSE;
  prev_rxrad_half_state.txspi_pkt_type = 255;
}

#endif 



