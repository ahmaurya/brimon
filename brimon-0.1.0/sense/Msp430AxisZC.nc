/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

generic configuration Msp430AxisZC() {
  provides interface Read<uint16_t>;
  provides interface ReadStream<uint16_t>;
  provides interface Resource;
  provides interface ReadNow<uint16_t>;
}
implementation {
  components new AdcReadClientC();
  Read = AdcReadClientC;

  components new AdcReadStreamClientC();
  ReadStream = AdcReadStreamClientC;

  components Msp430AxisZP;
  AdcReadClientC.AdcConfigure -> Msp430AxisZP;
  AdcReadStreamClientC.AdcConfigure -> Msp430AxisZP;

  components new AdcReadNowClientC();
  Resource = AdcReadNowClientC;
  ReadNow = AdcReadNowClientC;

  AdcReadNowClientC.AdcConfigure -> Msp430AxisZP;
}
