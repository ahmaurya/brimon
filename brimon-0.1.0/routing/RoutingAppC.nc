/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Jeet Patani
 * Copyright (C) 2011 Jeet Patani
 */

configuration RoutingAppC {}

implementation {

	components MainC, RoutingAppP as App, LedsC;
	components ActiveMessageC;
	components RoutingC;
	components new TimerMilliC();

	App.Boot -> MainC.Boot;
  
  	App.AMControl -> ActiveMessageC;
  	App.Leds -> LedsC;
	App.MilliTimer -> TimerMilliC;
	App.Routing -> RoutingC;
}

