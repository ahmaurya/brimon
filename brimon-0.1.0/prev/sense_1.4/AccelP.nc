/* $Id: AccelP.nc,v 1.3 2007/03/14 03:25:05 pipeng Exp $
 * Copyright (c) 2006 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */

/**
 * ADXL202JE accelerometer ADC configuration and power management.
 * @author David Gay <david.e.gay@intel.com>
 */

#include "Sense.h"
#include "Msp430Adc12.h"

configuration AccelP
{
  provides {
    interface AdcConfigure<const msp430adc12_channel_config_t*> as ConfigX;
    interface AdcConfigure<const msp430adc12_channel_config_t*> as ConfigY;
    interface AdcConfigure<const msp430adc12_channel_config_t*> as ConfigZ;
  }
}

implementation
{
	components Msp430AxisXP, Msp430AxisYP, Msp430AxisZP;
	ConfigX = Msp430AxisXP;
	ConfigY = Msp430AxisYP;
	ConfigZ = Msp430AxisZP;
}
