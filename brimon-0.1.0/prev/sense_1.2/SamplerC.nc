/*
 * Copyright (c) 2007-2009 Intel Corporation
 * All rights reserved.

 * This file is distributed under the terms in the attached INTEL-LICENS
 * file. If you do not find this file, a copy can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */

#include "Sense.h"

module SamplerC
{
  provides interface Sample;
  uses interface ReadStream<uint16_t> as AccelX;
  uses interface ReadStream<uint16_t> as AccelY;
  uses interface ReadStream<uint16_t> as AccelZ;
  uses interface BlockWrite as BlockWriteX;
  uses interface BlockWrite as BlockWriteY;
  uses interface BlockWrite as BlockWriteZ;
  uses interface Leds;
}
implementation
{
  uint8_t numEraseDone=0;
  uint8_t numSyncDone=0;

  uint16_t bufferx1[BUFFER_SIZE], bufferx2[BUFFER_SIZE], avgbufferx1[SUMMARY_BUFFER_SIZE], avgbufferx2[SUMMARY_BUFFER_SIZE];
  int8_t buffersxn; // how many buffers have been filled

  uint16_t buffery1[BUFFER_SIZE], buffery2[BUFFER_SIZE], avgbuffery1[SUMMARY_BUFFER_SIZE], avgbuffery2[SUMMARY_BUFFER_SIZE];
  int8_t buffersyn; // how many buffers have been filled

  uint16_t bufferz1[BUFFER_SIZE], bufferz2[BUFFER_SIZE], avgbufferz1[SUMMARY_BUFFER_SIZE], avgbufferz2[SUMMARY_BUFFER_SIZE];
  int8_t bufferszn; // how many buffers have been filled

  void startRead();

  command void Sample.sample() {
    // Sampling requested, start by erasing the block
    call BlockWriteX.erase();
    call BlockWriteY.erase();
    call BlockWriteZ.erase();
  }

  event void BlockWriteX.eraseDone(error_t ok) {
    call Leds.led2On();
    // Block erased. Post both buffers and initiate sampling
    call AccelX.postBuffer(bufferx1, BUFFER_SIZE);
    call AccelX.postBuffer(bufferx2, BUFFER_SIZE);
    buffersxn = 0;
	numEraseDone++;
	if(numEraseDone==3) {
		numEraseDone=0;
		startRead();
	}
  }

  event void BlockWriteY.eraseDone(error_t ok) {
    call Leds.led2On();
    // Block erased. Post both buffers and initiate sampling
    call AccelY.postBuffer(buffery1, BUFFER_SIZE);
    call AccelY.postBuffer(buffery2, BUFFER_SIZE);
    buffersyn = 0;
    numEraseDone++;
	if(numEraseDone==3) {
		numEraseDone=0;
		startRead();
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
		startRead();
	}
  }

  void startRead() {
	atomic {
		call AccelX.read(SAMPLE_PERIOD);
		call AccelY.read(SAMPLE_PERIOD);
		call AccelZ.read(SAMPLE_PERIOD);
	}
  }

  event void AccelX.bufferDone(error_t ok, uint16_t *buf, uint16_t count) {
    // A buffer is full. Write it to the block
    //call Leds.led2Toggle();
    uint32_t sum = 0;
    uint16_t i, j, k = 0;
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


  }

  event void AccelY.bufferDone(error_t ok, uint16_t *buf, uint16_t count) {
    // A buffer is full. Write it to the block
    call Leds.led2Toggle();
    call BlockWriteY.write(buffersyn * sizeof buffery1, buf, sizeof buffery1);
  }

  event void AccelZ.bufferDone(error_t ok, uint16_t *buf, uint16_t count) {
    // A buffer is full. Write it to the block
    call Leds.led2Toggle();
    call BlockWriteZ.write(bufferszn * sizeof bufferz1, buf, sizeof bufferz1);
  }

  event void BlockWriteX.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
				error_t error) {
    // Buffer written. TOTAL_SAMPLES is a multiple of BUFFER_SIZE, so
    // once we've posted TOTAL_SAMPLES / BUFFER_SIZE buffers we're done.
    // As we started by posting two buffers, the test below includes a -2
    if (++buffersxn <= TOTAL_SAMPLES / BUFFER_SIZE - 2)
      call AccelX.postBuffer(buf, BUFFER_SIZE);
    else if (buffersxn == TOTAL_SAMPLES / BUFFER_SIZE)
      // Once we've written all the buffers, flush writes to the buffer
      call BlockWriteX.sync();
  }

  event void BlockWriteY.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
				error_t error) {
    // Buffer written. TOTAL_SAMPLES is a multiple of BUFFER_SIZE, so
    // once we've posted TOTAL_SAMPLES / BUFFER_SIZE buffers we're done.
    // As we started by posting two buffers, the test below includes a -2
    if (++buffersyn <= TOTAL_SAMPLES / BUFFER_SIZE - 2)
      call AccelY.postBuffer(buf, BUFFER_SIZE);
    else if (buffersyn == TOTAL_SAMPLES / BUFFER_SIZE)
      // Once we've written all the buffers, flush writes to the buffer
      call BlockWriteY.sync();
  }

  event void BlockWriteZ.writeDone(storage_addr_t addr, void* buf, storage_len_t len,
				error_t error) {
    // Buffer written. TOTAL_SAMPLES is a multiple of BUFFER_SIZE, so
    // once we've posted TOTAL_SAMPLES / BUFFER_SIZE buffers we're done.
    // As we started by posting two buffers, the test below includes a -2
    if (++bufferszn <= TOTAL_SAMPLES / BUFFER_SIZE - 2)
      call AccelZ.postBuffer(buf, BUFFER_SIZE);
    else if (bufferszn == TOTAL_SAMPLES / BUFFER_SIZE)
      // Once we've written all the buffers, flush writes to the buffer
      call BlockWriteZ.sync();
  }

  event void BlockWriteX.syncDone(error_t error) {
    call Leds.led2Off();
	numSyncDone++;
	if(numSyncDone==3) {
		numSyncDone=0;
		signal Sample.sampled(error);
		}
  }

  event void BlockWriteY.syncDone(error_t error) {
    call Leds.led2Off();
	numSyncDone++;
	if(numSyncDone==3) {
		numSyncDone=0;
		signal Sample.sampled(error);
		}
  }

  event void BlockWriteZ.syncDone(error_t error) {
    call Leds.led2Off();
	numSyncDone++;
	if(numSyncDone==3) {
		numSyncDone=0;
		signal Sample.sampled(error);
		}
  }

  event void AccelX.readDone(error_t ok, uint32_t usActualPeriod) {
    // If we didn't use all buffers something went wrong, e.g., flash writes were
    // too slow, so the buffers did not get reposted in time
    signal Sample.sampled(FAIL);
  }

  event void AccelY.readDone(error_t ok, uint32_t usActualPeriod) {
    // If we didn't use all buffers something went wrong, e.g., flash writes were
    // too slow, so the buffers did not get reposted in time
    signal Sample.sampled(FAIL);
  }

  event void AccelZ.readDone(error_t ok, uint32_t usActualPeriod) {
    // If we didn't use all buffers something went wrong, e.g., flash writes were
    // too slow, so the buffers did not get reposted in time
    signal Sample.sampled(FAIL);
  }
}
