/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#include "Sense.h"

module SenseP
{
  provides interface Sense;

  uses interface Timer<TMilli>;
  uses interface Leds;
  uses interface LogWrite;
  uses interface SplitControl as RadioControl;

  uses interface Resource as ResourceX;
  uses interface Resource as ResourceY;
  uses interface Resource as ResourceZ;

  uses interface ReadNow<uint16_t> as AccelX;
  uses interface ReadNow<uint16_t> as AccelY;
  uses interface ReadNow<uint16_t> as AccelZ;
}

implementation
{
  uint8_t state = NOT_ERASED;
  uint8_t bufferIndex = B1;
  uint16_t bufferSampleIndex = 0;
  uint16_t globalSampleIndex = 0;
  uint16_t globalBufferIndex = 0;
  uint32_t bufferTimer;
  uint32_t bufferTimerDiff;
  uint32_t logStartOffset;

  uint16_t buffer[NUM_BUFFER][NUM_AXIS][BUFFER_SIZE];
  uint16_t bufferStartTimers[TOTAL_BUFFERS];
  struct sample samples[SUMMARY_BUFFER_SIZE];

  void startSampling();
  void sample();
  task void checkCompletion();
  task void doCompletion();
  void summarize(uint8_t x);
  void eraseLog();
  void resetParameters();

  void resetParameters() {
  atomic {
	bufferIndex = B1;
	bufferSampleIndex = 0;
	globalSampleIndex = 0;
	globalBufferIndex = 0;
	}
  }

  command void Sense.sense() {
    // Sampling requested, decide course of action based on current state
    switch(state) {
	case NOT_ERASED:
		atomic state = ERASING;
		eraseLog();
		signal Sense.senseDone(ERASING);
		break;
	case ERASING:
		signal Sense.senseDone(ERASING);
		break;
	case ERASED:
		atomic state = SAMPLING;
		startSampling();
		break;
	case SAMPLING:
		signal Sense.senseDone(SAMPLING);
		break;
	case SUMMARIZING:
		signal Sense.senseDone(SUMMARIZING);
		break;
    }
  }

  command uint16_t Sense.getLogSize() {
	return TOTAL_BUFFERS*(sizeof(bufferTimer) + sizeof(samples));
  }

  void eraseLog() {
	//call LogWrite.erase();
	signal LogWrite.eraseDone(SUCCESS);
  }

  event void LogWrite.appendDone(void* buf, storage_len_t len, bool recordsLost, error_t error) {
	printf("Writing done: %u\n\n", globalBufferIndex - 1);
	printfflush();
  }

  event void LogWrite.eraseDone(error_t error) {
	atomic state = ERASED;
	call Leds.set(2);
  }

  event void LogWrite.syncDone(error_t error) {
    // Summary saved!
    call Leds.set(7);
	atomic state = ERASING;
	eraseLog();
	signal Sense.senseDone(error);
	//call RadioControl.start();
  }

  event void RadioControl.startDone(error_t err) {}

  event void RadioControl.stopDone(error_t err) {}

  void startSampling() {
	resetParameters();
	//call RadioControl.stop();
	call Leds.set(2);
	logStartOffset = call LogWrite.currentOffset();
	call Timer.startPeriodic(SAMPLE_PERIOD);
  }

  event void Timer.fired() {
	if(bufferSampleIndex == 0) {
		bufferTimerDiff = (call Timer.getNow()) - bufferTimer;
		bufferTimer = call Timer.getNow();
		}
	sample();
  }

  void sample() {
	call ResourceX.request();
  }

  event void ResourceX.granted() {
	call AccelX.read();
  }

  async event void AccelX.readDone( error_t result, uint16_t val ) {
		buffer[bufferIndex][X][bufferSampleIndex] = val;
		atomic {
			call ResourceX.release();
			call ResourceY.request();
			}
  }

  event void ResourceY.granted() {
	call AccelY.read();
  }

  async event void AccelY.readDone( error_t result, uint16_t val ) {
		buffer[bufferIndex][Y][bufferSampleIndex] = val;
		atomic {
			call ResourceY.release();
			call ResourceZ.request();
			}
  }

  event void ResourceZ.granted() {
	call AccelZ.read();
  }

  async event void AccelZ.readDone( error_t result, uint16_t val ) {
		buffer[bufferIndex][Z][bufferSampleIndex] = val;
		atomic {
			call ResourceZ.release();
			post checkCompletion();
			}
  }

  task void checkCompletion() {
	globalSampleIndex++;
	if(globalSampleIndex == TOTAL_SAMPLES)
		post doCompletion();
	atomic bufferSampleIndex++;
	if(bufferSampleIndex == BUFFER_SIZE) {
		summarize(bufferIndex);
		atomic bufferIndex = (bufferIndex+1)%NUM_BUFFER;
		globalBufferIndex++;
		atomic bufferSampleIndex = 0;
	}
  }

  task void doCompletion() {
	call Timer.stop();
	//call Timer.startPeriodic(999999999L);
	//call LogWrite.sync();
  }

  void summarize(uint8_t bufferIndexToSum) {
    uint32_t sumX = 0, sumY = 0, sumZ = 0, temp;
    uint16_t i, j, k = 0;
    call Leds.set(7);
	for (i = 0; i < SUMMARY_BUFFER_SIZE; i++) {
		sumX = sumY = sumZ = 0;
		for (j = 0; j < DFACTOR; j++) {
		atomic {
			sumX += buffer[bufferIndexToSum][X][k];
			sumY += buffer[bufferIndexToSum][Y][k];
			sumZ += buffer[bufferIndexToSum][Z][k];
			k++;
			}
		}
		samples[i].x = sumX / DFACTOR;
		samples[i].y = sumY / DFACTOR;
		samples[i].z = sumZ / DFACTOR;
		printf("%u ... %u ... %u ... %u\n\n", i, samples[i].x, samples[i].y, samples[i].z); printfflush();
	}
	call LogWrite.append(&bufferTimer, sizeof bufferTimer);
	call LogWrite.append(samples, sizeof samples);
	printf("%lu :: ", bufferTimer);
	temp = bufferTimer-1;
	printf("%lu :: ", temp);
	temp = bufferTimer+bufferTimerDiff;
	printf("%lu :: ", temp);
	printf("%lu :: ", bufferTimerDiff);
	printf("writing the %u buffer to the logfile\n\n", globalBufferIndex);
	printfflush();
	call Leds.set(1);
	if(globalBufferIndex == TOTAL_BUFFERS - 1)
		//call LogWrite.sync();
		signal LogWrite.syncDone(SUCCESS);
  }
}
