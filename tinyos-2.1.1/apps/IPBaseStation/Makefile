COMPONENT=BaseStationC

# radio options.  channel settings will be overridden by the driver
# CFLAGS += -DCC2420_DEF_RFPOWER=4

# this is necessary, otherwise we will allocate a heap by including
# the lowpan target
CFLAGS += -DNO_IP_MALLOC 

# this aligns the 802.15.4 payload on the data field of a message_t.
# very convenient for forwarding, and means we don't have to do a
# memmove.  This doesn't mean that motes running blip also need this--
# they can use IFRAMES or IEEE154FRAMES
CFLAGS += -DIEEE154FRAMES_ENABLED

# if you're worried about your interface mote crashing, enable this to
# turn on a hardware watchdog that will reset it if it does.  CFLAGS
# += -DBLIP_WATCHDOG
#
# debugging
#

# filter packets based on a specified topology
# PFLAGS += -I../UDPEcho/NodeConnectivity -DSW_TOPOLOGY

# part of the test harness
# CFLAGS += -DDBG_TRACK_FLOWS

# for simulation
# CFLAGS += -DBASESTATION_ID=100

# defs for snooping-- handy for debugging LPL, etc.
# CFLAGS += -DCC2420_NO_UNIQUE
# CFLAGS += -DCC2420_DEF_CHANNEL=15
# CFLAGS += -DCC2420_NO_ACKNOWLEDGEMENTS
# CFLAGS += -DCC2420_NO_ADDRESS_RECOGNITION

include $(MAKERULES)

