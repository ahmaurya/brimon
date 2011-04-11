/*
 * Copyright (c) 2007-2009 Intel Corporation
 * All rights reserved.

 * This file is distributed under the terms in the attached INTEL-LICENS
 * file. If you do not find this file, a copy can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */

/*
 * Modified by Abhinav Maurya
 */

#ifndef FLASHSAMPLER_H
#define FLASHSAMPLER_H

enum {
	SAMPLE_INTERVAL = 1024L * 60,
		//periodic interval after which sampling is initiated
	SAMPLE_PERIOD = 5,
		//period of sampling operation

	BUFFER_SIZE = 400,
		// samples per buffer
	TOTAL_SAMPLES = 8000,
		// must be multiple of BUFFER_SIZE
	TOTAL_BUFFERS = TOTAL_SAMPLES / BUFFER_SIZE,
		// total number of buffers

	SUMMARY_SAMPLES = 1600,
		// total samples in summary
	DFACTOR = TOTAL_SAMPLES / SUMMARY_SAMPLES,
		// downsampling factor: real samples per summary sample
	SUMMARY_BUFFER_SIZE = BUFFER_SIZE / DFACTOR,
		// size of a buffer summary

	VOLUME_SAMPLELOG = 100,
		// volume id for LogStorageC
	VOLUME_SAMPLES_X = 101,
		// volume id for BlockStorageXC
	VOLUME_SAMPLES_Y = 102,
		// volume id for BlockStorageYC
	VOLUME_SAMPLES_Z = 103
		// volume id for BlockStorageZC
	};

struct sample{
	uint16_t x:12;
	uint16_t y:12;
	uint16_t z:12;
} sample_t;

#endif
