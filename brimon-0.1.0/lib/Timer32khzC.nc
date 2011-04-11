/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

generic configuration Timer32khzC() {
	provides interface Timer<T32khz>;
}

implementation {
	components new Alarm32khz32C() as AlarmC;
	components new AlarmToTimerC(T32khz);

	AlarmToTimerC.Alarm -> AlarmC;
	Timer = AlarmToTimerC;
}
