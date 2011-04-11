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
	SAMPLE_PERIOD = 1000,
		//period of sampling operation

	BUFFER_SIZE = 512,
		// samples per buffer
	TOTAL_SAMPLES = 32768L,
		// must be multiple of BUFFER_SIZE
	TOTAL_BUFFERS = TOTAL_SAMPLES / BUFFER_SIZE,
		// total number of buffers

	SUMMARY_SAMPLES = 256,
		// total samples in summary
	DFACTOR = 32768L / SUMMARY_SAMPLES
		// downsampling factor: real samples per summary sample
	};

#endif
