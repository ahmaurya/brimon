/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

#ifndef BRIMONAPP_H
#define BRIMONAPP_H

#include "StorageVolumes.h"

enum {
	SAMPLE_INTERVAL = 1024L * 10,
		//periodic interval after which sampling is initiated
	SAMPLE_PERIOD = 2,
		//period of sampling operation

	NUM_AXIS = 3,
		//number of axes
	X = 0, Y = 1, Z = 2,
		//indices of the axes in the buffer

	NUM_BUFFER = 2,
		//number of buffers
	B1 = 0, B2 = 1,
		//indices of the buffers

	BUFFER_SIZE = 400,
		// samples per buffer
	TOTAL_SAMPLES = 16000,
		// must be multiple of BUFFER_SIZE
	SUMMARY_SAMPLES = 1600,
		// total samples in summary

	TOTAL_BUFFERS = TOTAL_SAMPLES / BUFFER_SIZE,
		// total number of buffers
	DFACTOR = TOTAL_SAMPLES / SUMMARY_SAMPLES,
		// downsampling factor: real samples per summary sample
	SUMMARY_BUFFER_SIZE = BUFFER_SIZE / DFACTOR,
		// size of a buffer summary

	//VOLUME_SAMPLELOG = 100,
		// volume id for LogStorageC
	//VOLUME_SAMPLES_X = 101,
		// volume id for BlockStorageXC
	//VOLUME_SAMPLES_Y = 102,
		// volume id for BlockStorageYC
	//VOLUME_SAMPLES_Z = 103,
		// volume id for BlockStorageZC

	NOT_ERASED = 100,
	ERASING = 101,
	ERASED = 102,
	SAMPLING = 103,
	SUMMARIZING = 104
		// states of the SamplerC component
	};

struct sample{
	uint16_t x:12;
	uint16_t y:12;
	uint16_t z:12;
} sample_t;

#endif
