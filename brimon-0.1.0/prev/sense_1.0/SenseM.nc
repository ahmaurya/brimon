#include<printf.h>

module SenseM {
	uses {
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as TheftTimer;
		interface ReadStream<uint16_t> as Accel;
	}
}

implementation {

enum {
	ACCEL_INTERVAL = 256, /* Checking interval */
	ACCEL_PERIOD = 10000, /* uS -> 100Hz */
	ACCEL_NSAMPLES = 10,  /* 10 samples * 100Hz -> 0.1s */
	ACCEL_VARIANCE = 4
	};

	uint16_t accelSamples[ACCEL_NSAMPLES];

	task void checkAcceleration();

	event void Boot.booted() {
		call TheftTimer.startPeriodic(ACCEL_INTERVAL);
	}

	event void TheftTimer.fired() {
		// Get 10 samples at 100Hz
		call Accel.postBuffer(accelSamples, ACCEL_NSAMPLES);
		call Accel.read(ACCEL_INTERVAL);
	}

	/* The acceleration read completed. Post the task to check for theft */
	event void Accel.readDone(error_t ok, uint32_t usActualPeriod) {
		if (ok == SUCCESS)
		post checkAcceleration();
	}

	/* Check if acceleration variance above threshold */
	task void checkAcceleration() {
		uint8_t i;
		uint32_t avg, variance;
		for (avg = 0, i = 0; i < ACCEL_NSAMPLES; i++)
			avg += accelSamples[i];
		avg /= ACCEL_NSAMPLES;
		for (variance = 0, i = 0; i < ACCEL_NSAMPLES; i++)
			variance += (int16_t)(accelSamples[i] - avg) * (int16_t)(accelSamples[i] - avg);
		printf("Average: %u ... %u\n\n",avg,(avg)*3L/4096.0);
		if (variance > ACCEL_VARIANCE * ACCEL_NSAMPLES)
			call Leds.led2On(); /* ALERT! ALERT! */
		else
			call Leds.led2Off(); /* Don’t leave LED permanently on */
	}

	event void Accel.bufferDone(error_t ok, uint16_t *buf, uint16_t count) { }
}
