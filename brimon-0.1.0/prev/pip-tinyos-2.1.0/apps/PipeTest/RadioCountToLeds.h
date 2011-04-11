/*
 * "Copyright (c) 2004-2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */

#ifndef RADIO_COUNT_TO_LEDS_H
#define RADIO_COUNT_TO_LEDS_H

//#include "/home/br/svn/pip/pip-tinyos-2.1.0/tos/chips/cc2420/pipeline.h"
#include "pipeline.h"

//#define RCM_DATA_LENGTH 103
// 103 is TOSH_DATA_LENGTH-sizeof(seqno)-sizeof(num_entries), but 103 does not seem to
// work; value of 99 works
#define RCM_DATA_LENGTH 99
#define RADIO_LOG_DATA_LENGTH 89

/*** the payload of data_packet is mapped to following struct*/

typedef nx_struct radio_count_msg {
  nx_uint16_t seqno;
  // number of valid entries in the data[]; this field is used in SNACK
  nx_uint16_t num_entries;
  nx_uint8_t data[RCM_DATA_LENGTH];
} radio_count_msg_t;


/** Following struct is used for multi-hop log collection Currently,
    number of packets sent in during log collection = number of packets during exptal
    transfer even if number of log entries are less. So isValid flag
    has been added to ignore packets which dont contain valid log entries**/

typedef nx_struct RadioLog {
  nxle_uint32_t time;
  nxle_uint32_t value;
  nxle_uint8_t type;
  nxle_uint32_t seqno;
  nxle_uint8_t  isValid;
  nxle_uint8_t data[RADIO_LOG_DATA_LENGTH];  
} RadioLog;




enum {
  AM_RADIO_COUNT_MSG = 6,
};

/** PIP transport states as per protocol***/
typedef enum {
  PIP_TR_S_INIT,
  PIP_TR_S_DATASEND,
  PIP_TR_S_SNACKWAIT,
} pip_transport_state_t;

/** These are control msgs transferred betw mote and PC. ( This is
    control channel as explained by report) ***/
typedef enum {
  #include "control_msg_codes.h"
} control_msg_codes_t;

#endif
