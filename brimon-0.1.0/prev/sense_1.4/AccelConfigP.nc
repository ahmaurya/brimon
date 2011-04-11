/* $Id: AccelConfigP.nc,v 1.3 2007/03/14 03:25:05 pipeng Exp $
 * Copyright (c) 2006 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */

/**
 * Internal component for basicsb photodiode. Arbitrates access to the photo
 * diode and automatically turns it on or off based on user requests.
 *
 * @author David Gay
 */

#include "Sense.h"
#include "Msp430Adc12.h"

configuration AccelConfigP {
  provides {
    interface Resource[uint8_t client];
    interface AdcConfigure<const msp430adc12_channel_config_t*> as ConfigX;
    interface AdcConfigure<const msp430adc12_channel_config_t*> as ConfigY;
    interface AdcConfigure<const msp430adc12_channel_config_t*> as ConfigZ;
  }
}
implementation {
  components AccelP, new RoundRobinArbiterC(UQ_ACCEL_RESOURCE) as Arbiter;

  Resource = Arbiter;
  ConfigX = AccelP.ConfigX;
  ConfigY = AccelP.ConfigY;
  ConfigZ = AccelP.ConfigZ;
}
