/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

configuration TimeSyncAppC {}

implementation {
	components TimeSyncAppP as App;
	components TimeSyncC;
	components MainC, LedsC;
	components new TimerMilliC();
	components ActiveMessageC;

	App.Boot -> MainC.Boot;
	App.Leds -> LedsC;
	App.Timer -> TimerMilliC;
	App.TimeSync -> TimeSyncC;
	App.AMControl -> ActiveMessageC;
}
