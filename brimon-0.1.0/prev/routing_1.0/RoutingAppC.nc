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

