/*
 * Copyright (c) 2007-2009 Intel Corporation
 * All rights reserved.

 * This file is distributed under the terms in the attached INTEL-LICENS
 * file. If you do not find this file, a copy can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */

#include "Sense.h"
#include <printf.h>

module SamplerC
{
  provides interface Sample;
  uses interface Leds;
  uses interface Timer<TMilli>;

  uses interface ReadStream<uint16_t> as AccelX;
  uses interface ReadStream<uint16_t> as AccelY;
  uses interface ReadStream<uint16_t> as AccelZ;

  uses interface BlockRead as BlockReadX;
  uses interface BlockRead as BlockReadY;
  uses interface BlockRead as BlockReadZ;

  uses interface BlockWrite as BlockWriteX;
  uses interface BlockWrite as BlockWriteY;
  uses interface BlockWrite as BlockWriteZ;

  uses interface LogWrite;
}

implementation
{
  uint8_t numEraseDone = 0;
  uint8_t numSyncDone  = 0;
  uint8_t numReadDone = 0;
  uint8_t state = NOT_ERASED;
  //uint8_t blockStorageIndex, blockStoragePosX[TOTAL_BUFFERS], blockStoragePosY[TOTAL_BUFFERS], blockStoragePosZ[TOTAL_BUFFERS];

  uint16_t bufferx1[BUFFER_SIZE], bufferx2[BUFFER_SIZE];
  uint16_t avgbufferx1[SUMMARY_BUFFER_SIZE], avgbufferx2[SUMMARY_BUFFER_SIZE];
  uint8_t buffersxn = 0; // how many buffers have been filled

  uint16_t buffery1[BUFFER_SIZE], buffery2[BUFFER_SIZE];
  uint16_t avgbuffery1[SUMMARY_BUFFER_SIZE], avgbuffery2[SUMMARY_BUFFER_SIZE];
  uint8_t buffersyn = 0; // how many buffers have been filled

  uint16_t bufferz1[BUFFER_SIZE], bufferz2[BUFFER_SIZE];
  uint16_t avgbufferz1[SUMMARY_BUFFER_SIZE], avgbufferz2[SUMMARY_BUFFER_SIZE];
  uint8_t bufferszn = 0; // how many buffers have been filled

  //uint16_t bufferFinishTimersX[TOTAL_BUFFERS], bufferFinishTimersY[TOTAL_BUFFERS], bufferFinishTimersZ[TOTAL_BUFFERS];

  struct sample finalSamples[SUMMARY_BUFFER_SIZE];
  uint16_t finalSamplesIndex = 0;

  void startSampling();
  void summarize();
  void writeLog();
  void eraseBlocks();
  void resetParameters();

  void resetParameters() {
	numEraseDone = 0;
	numSyncDone  = 0;
	numReadDone = 0;
	state = NOT_ERASED;
	buffersxn = 0;
	buffersyn = 0;
	bufferszn = 0;
	finalSamplesIndex = 0;
  }

  command void Sample.sample() {
    // Sampling requested, decide course of action based on current state
    switch(state) {
	case NOT_ERASED:
		atomic state = ERASING;
		eraseBlocks();
		signal Sample.sampled(ERASING);
		break;
	case ERASING:
		signal Sample.sampled(ERASING);
		break;
	case ERASED:
		atomic state = SAMPLING;
		startSampling();
		break;
	case SAMPLING:
		signal Sample.sampled(SAMPLING);
		break;
	case SUMMARIZING:
		signal Sample.sampled(SUMMARIZING);
		break;
    }
  }

  event void Timer.fired() {}

  void eraseBlocks() {
	call BlockWriteX.erase();
    call BlockWriteY.erase();
    call BlockWriteZ.erase();
    call LogWrite.erase();
  }

  event void BlockWriteX.eraseDone(error_t ok) {
    call Leds.led0On();
    // Block erased. Post both buffers and initiate sampling
    call AccelX.postBuffer(bufferx1, BUFFER_SIZE);
    call AccelX.postBuffer(bufferx2, BUFFER_SIZE);
    buffersxn = 0;
	numEraseDone++;
	if(numEraseDone==3) {
		numEraseDone=0;
		//startSampling();
		atomic state = ERASED;
	}
  }

  event void BlockWriteY.eraseDone(error_t ok) {
    call Leds.led1On();
    // Block erased. Post both buffers and initiate sampling
    call AccelY.postBuffer(buffery1, BUFFER_SIZE);
    call AccelY.postBuffer(buffery2, BUFFER_SIZE);
    buffersyn = 0;
    numEraseDone++;
	if(numEraseDone==3) {
		numEraseDone=0;
		//startSampling();
		atomic state = ERASED;
	}
  }

  event void BlockWriteZ.eraseDone(error_t ok) {
    call Leds.led2On();
    // Block erased. Post both buffers and initiate sampling
    call AccelZ.postBuffer(bufferz1, BUFFER_SIZE);
    call AccelZ.postBuffer(bufferz2, BUFFER_SIZE);
    bufferszn = 0;
    numEraseDone++;
	if(numEraseDone==3) {
		numEraseDone=0;
		//startSampling();
		atomic state = ERASED;
	}
  }

  void startSampling() {
		//call LogWrite.erase();
		call AccelX.read(SAMPLE_PERIOD);
		call AccelY.read(SAMPLE_PERIOD);
		call AccelZ.read(SAMPLE_PERIOD);
  }

  event void AccelX.bufferDone(error_t ok, uint16_t *buf, uint16_t count) {
    // A buffer is full. Write it to the block
    uint32_t sum = 0;
    uint16_t i, j, k = 0;
    call Leds.led0Toggle();
    printf("Summarizing and writing to BlockStorage buffer # %u for axis X\n\n", buffersxn); printfflush();
    if(buf==bufferx1) {
		for (i = 0; i < SUMMARY_BUFFER_SIZE; i++) {
			sum = 0;
			for (j = 0; j < DFACTOR; j++) {
				sum += bufferx1[k];
				k++;
			}
			avgbufferx1[i] = sum / DFACTOR;
		}
		call BlockWriteX.write(buffersxn * sizeof avgbufferx1, avgbufferx1, sizeof avgbufferx1);
    }
    if(buf==bufferx2) {
		for (i = 0; i < SUMMARY_BUFFER_SIZE; i++) {
			sum = 0;
			for (j = 0; j < DFACTOR; j++) {
				sum += bufferx2[k];
				k++;
			}
			avgbufferx2[i] = sum / DFACTOR;
		}
		call BlockWriteX.write(buffersxn * sizeof avgbufferx2, avgbufferx2, sizeof avgbufferx2);
    }
  }

  event void AccelY.bufferDone(error_t ok, uint16_t *buf, uint16_t count) {
    // A buffer is full. Write it to the block
    uint32_t sum = 0;
    uint16_t i, j, k = 0;
    call Leds.led1Toggle();
    printf("Summarizing and writing to BlockStorage buffer # %u for axis Y\n\n", buffersyn);  printfflush();
    if(buf==buffery1) {
		for (i = 0; i < SUMMARY_BUFFER_SIZE; i++) {
			sum = 0;
			for (j = 0; j < DFACTOR; j++) {
				sum += buffery1[k];
				k++;
			}
			avgbuffery1[i] = sum / DFACTOR;
		}
		call BlockWriteY.write(buffersyn * sizeof avgbuffery1, avgbuffery1, sizeof avgbuffery1);
    }
    if(buf==buffery2) {
		for (i = 0; i < SUMMARY_BUFFER_SIZE; i++) {
			sum = 0;
			for (j = 0; j < DFACTOR; j++) {
				sum += buffery2[k];
				k++;
			}
			avgbuffery2[i] = sum / DFACTOR;
		}
		call BlockWriteY.write(buffersyn * sizeof avgbuffery2, avgbuffery2, sizeof avgbuffery2);
    }
  }

  event void AccelZ.bufferDone(error_t ok, uint16_t *buf, uint16_t count) {
    // A buffer is full. Write it to the block
    uint32_t sum = 0;
    uint16_t i, j, k = 0;
    call Leds.led2Toggle();
    printf("Summarizing and writing to BlockStorage buffer # %u for axis Z\n\n", bufferszn);  printfflush();
    if(buf==bufferz1) {
		for (i = 0; i < SUMMARY_BUFFER_SIZE; i++) {
			sum = 0;
			for (j = 0; j < DFACTOR; j++) {
				sum += bufferz1[k];
				k++;
			}
			avgbufferz1[i] = sum / DFACTOR;
		}
		call BlockWriteZ.write(bufferszn * sizeof avgbufferz1, avgbufferz1, sizeof avgbufferz1);
    }
    if(buf==bufferz2) {
		for (i = 0, k = 0; i < SUMMARY_BUFFER_SIZE; i++) {
			sum = 0;
			for (j = 0; j < DFACTOR; j++) {
				sum += bufferz2[k];
				k++;
			}
			avgbufferz2[i] = sum / DFACTOR;
		}
		call BlockWriteZ.write(bufferszn * sizeof avgbufferz2, avgbufferz2, sizeof avgbufferz2);
    }
  }

  event void BlockWriteX.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
				error_t error) {
    // Buffer written. TOTAL_SAMPLES is a multiple of BUFFER_SIZE, so
    // once we've posted TOTAL_SAMPLES / BUFFER_SIZE buffers we're done.
    // As we started by posting two buffers, the test below includes a -2
    //bufferFinishTimersX[buffersxn] = call Timer.getNow();
    if (++buffersxn <= TOTAL_BUFFERS - 2)
      call AccelX.postBuffer(buf==avgbufferx1?bufferx1:bufferx2, BUFFER_SIZE);
    else if (buffersxn == TOTAL_BUFFERS)
      // Once we've written all the buffers, flush writes to the buffer
      call BlockWriteX.sync();
  }

  event void BlockWriteY.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
				error_t error) {
    // Buffer written. TOTAL_SAMPLES is a multiple of BUFFER_SIZE, so
    // once we've posted TOTAL_SAMPLES / BUFFER_SIZE buffers we're done.
    // As we started by posting two buffers, the test below includes a -2
    //summarize();
    if (++buffersyn <= TOTAL_BUFFERS - 2)
      call AccelY.postBuffer(buf==avgbuffery1?buffery1:buffery2, BUFFER_SIZE);
    else if (buffersyn == TOTAL_BUFFERS)
      // Once we've written all the buffers, flush writes to the buffer
      call BlockWriteY.sync();
  }

  event void BlockWriteZ.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
				error_t error) {
    // Buffer written. TOTAL_SAMPLES is a multiple of BUFFER_SIZE, so
    // once we've posted TOTAL_SAMPLES / BUFFER_SIZE buffers we're done.
    // As we started by posting two buffers, the test below includes a -2
    if (++bufferszn <= TOTAL_BUFFERS - 2)
      call AccelZ.postBuffer(buf==avgbufferz1?bufferz1:bufferz2, BUFFER_SIZE);
    else if (bufferszn == TOTAL_BUFFERS)
      // Once we've written all the buffers, flush writes to the buffer
      call BlockWriteZ.sync();
  }

  event void BlockWriteX.syncDone(error_t error) {
    call Leds.led0On();
    printf("Syncing BlockStorage of axis X\n\n");  printfflush();
	numSyncDone++;
	if(numSyncDone==3) {
		numSyncDone=0;
		//signal Sample.sampled(error);
		summarize();
		}
  }

  event void BlockWriteY.syncDone(error_t error) {
    call Leds.led1On();
    printf("Syncing BlockStorage of axis Y\n\n");  printfflush();
	numSyncDone++;
	if(numSyncDone==3) {
		numSyncDone=0;
		//signal Sample.sampled(error);
		summarize();
		}
  }

  event void BlockWriteZ.syncDone(error_t error) {
    call Leds.led2On();
    printf("Syncing BlockStorage of axis Z\n\n");  printfflush();
	numSyncDone++;
	if(numSyncDone==3) {
		numSyncDone=0;
		//signal Sample.sampled(error);
		summarize();
		}
  }

  event void AccelX.readDone(error_t ok, uint32_t usActualPeriod) {
    // If we didn't use all buffers something went wrong, e.g., flash writes were
    // too slow, so the buffers did not get reposted in time
    printf("Failure due to readDone of axis X\n\n");  printfflush();
    resetParameters();
    signal Sample.sampled(FAIL);
  }

  event void AccelY.readDone(error_t ok, uint32_t usActualPeriod) {
    // If we didn't use all buffers something went wrong, e.g., flash writes were
    // too slow, so the buffers did not get reposted in time
    printf("Failure due to readDone of axis Y\n\n");  printfflush();
    resetParameters();
    signal Sample.sampled(FAIL);
  }

  event void AccelZ.readDone(error_t ok, uint32_t usActualPeriod) {
    // If we didn't use all buffers something went wrong, e.g., flash writes were
    // too slow, so the buffers did not get reposted in time
    printf("Failure due to readDone of axis Z\n\n");  printfflush();
    resetParameters();
    signal Sample.sampled(FAIL);
  }

  void summarize() {
	printf("Starting summarizing\n\n");  printfflush();
	atomic state = SUMMARIZING;
	//collate accelerometer readings for the three axes
	call BlockReadX.read(finalSamplesIndex * sizeof avgbufferx1, avgbufferx1, sizeof avgbufferx1);
	call BlockReadY.read(finalSamplesIndex * sizeof avgbuffery1, avgbuffery1, sizeof avgbuffery1);
	call BlockReadZ.read(finalSamplesIndex * sizeof avgbufferz1, avgbufferz1, sizeof avgbufferz1);
  }

  event void BlockReadX.readDone(storage_addr_t addr, void* buf, storage_len_t len,
				error_t error) {
	call Leds.led0On();
	numReadDone++;
	if(numReadDone==3) {
		numReadDone=0;
		writeLog();
	}
  }

  event void BlockReadY.readDone(storage_addr_t addr, void* buf, storage_len_t len,
				error_t error) {
	call Leds.led1On();
	numReadDone++;
	if(numReadDone==3) {
		numReadDone=0;
		writeLog();
	}
  }

  event void BlockReadZ.readDone(storage_addr_t addr, void* buf, storage_len_t len,
				error_t error) {
	call Leds.led2On();
	numReadDone++;
	if(numReadDone==3) {
		numReadDone=0;
		writeLog();
	}
  }

  event void BlockReadX.computeCrcDone(storage_addr_t addr, storage_len_t len, uint16_t crc, error_t error) {}

  event void BlockReadY.computeCrcDone(storage_addr_t addr, storage_len_t len, uint16_t crc, error_t error) {}

  event void BlockReadZ.computeCrcDone(storage_addr_t addr, storage_len_t len, uint16_t crc, error_t error) {}

  void writeLog() {
	//write out results to log
	uint16_t i=0;

	//copy everything read into summary buffers into struct sample
	//this involves compression using 12-bit bit-fields for each axis reading of each summarized sample
	for(i=0; i<SUMMARY_BUFFER_SIZE; i++) {
		finalSamples[i].x=avgbufferx1[i];
		finalSamples[i].y=avgbuffery1[i];
		finalSamples[i].z=avgbufferz1[i];
	}

	//write out finalSamples to logfile
	call LogWrite.append(finalSamples, sizeof finalSamples);

	//increment finalSamplesIndex by the number of samples read in this iteration
	//finalSamplesIndex += SUMMARY_BUFFER_SIZE;
	finalSamplesIndex++;

    // Move on to the next block of summary samples
    if (finalSamplesIndex < TOTAL_BUFFERS)
		summarize();
    else {
		//reset finalSamplesIndex to zero
		finalSamplesIndex = 0;
		//sync the logfile
		call LogWrite.sync();
    }
  }

  event void LogWrite.appendDone(void* buf, storage_len_t len, bool recordsLost, error_t error) {
	call Leds.set(0);
  }

  event void LogWrite.eraseDone(error_t error) {}

  event void LogWrite.syncDone(error_t error) {
    // Summary saved!
	//initiate an erase of BlockStorages for next sampling
	atomic state = ERASING;
	eraseBlocks();
    call Leds.set(7);
    signal Sample.sampled(error);
  }
}
