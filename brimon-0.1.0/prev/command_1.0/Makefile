# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# Written (W) 2011 Abhinav Maurya
# Copyright (C) 2011 Abhinav Maurya

COMPONENT=CommandAppC

BUILD_EXTRA_DEPS += Serial.class
CLEAN_EXTRA = *.class SerialMsg.java

CFLAGS += -I$(TOSDIR)/lib/T2Hack

Serial.class: $(wildcard *.java) SerialMsg.java
	javac -target 1.4 -source 1.4 *.java

SerialMsg.java:
	mig java -target=null $(CFLAGS) -java-classname=SerialMsg Serial.h test_serial_msg -o $@

#CFLAGS += -I$(TOSDIR)/lib/printf
CFLAGS += -I$(APPROOT)/../lib/
CFLAGS += -I$(APPROOT)/../sense
CFLAGS += -I$(APPROOT)/../routing
CFLAGS += -I$(APPROOT)/../transfer

include $(MAKERULES)

