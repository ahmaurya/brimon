//PC-TO-MOTE  OP_CODES
START_ROUTING,
GET_ROUTING_LOG,
GET_ROUTING_TREE,
RESET_RTG_STATE,
CONFIGURE_CONN,
START_TRANSFER,
GET_PATH_OF_SRC,
COLLECT_LOG,

// Following  OP_CODES are  used to display menu. Not for MOTE_PC Communication


PARSE_LOG,
PARSE_ROUTING_LOG,
EXIT,
END_OP_CODES,
// Till here it will appear in menu

//MOTE-TO-PC CONTROL MSGS 
NOOP,
SENT_ROUTING_TREE,
TRANSFER_STARTED,
PATH_SENT,
DONE_COLLECTING,
DONE_ROUTING,
RTG_RESET_DONE,
UNKNOWN_COMMAND,


