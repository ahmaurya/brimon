/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "EventSignal.h"

configuration EventSignalAppC {}

implementation {
  components MainC, LedsC;
  components EventSignalAppP as App;
  components new AMSenderC(AM_RADIO_BEACON_MSG);
  components CC2420TimeSyncMessageC;
  components new TimerMilliC();
  components ActiveMessageC, CC2420ControlC;

  App.Boot -> MainC.Boot;
  App.Leds -> LedsC;
  App.MilliTimer -> TimerMilliC;
  App.AMSend -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  App.Packet -> CC2420TimeSyncMessageC.Packet;
  App.CC2420Config -> CC2420ControlC;
}
