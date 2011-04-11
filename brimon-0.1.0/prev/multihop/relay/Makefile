MSG_SIZE = 102
COMPONENT=RadioCountToLedsAppC
BUILD_EXTRA_DEPS = RadioCountMsg.py RadioCountMsg.class
CLEAN_EXTRA = RadioCountMsg.py RadioCountMsg.class RadioCountMsg.java

RadioCountMsg.py: RadioCountToLeds.h
	mig python -target=$(PLATFORM) $(CFLAGS) -python-classname=RadioCountMsg RadioCountToLeds.h radio_count_msg -o $@

RadioCountMsg.class: RadioCountMsg.java
	javac RadioCountMsg.java

RadioCountMsg.java: RadioCountToLeds.h
	mig java -target=$(PLATFORM) $(CFLAGS) -java-classname=RadioCountMsg RadioCountToLeds.h radio_count_msg -o $@


include $(MAKERULES)
CFLAGS += -I$(TOSDIR)/lib/printf
CFLAGS += -I/opt/tinyos-2.1.1/apps/prasanna_app/sender/
