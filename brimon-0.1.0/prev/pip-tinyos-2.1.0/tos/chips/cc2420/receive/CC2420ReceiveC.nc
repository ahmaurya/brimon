/*
 * Copyright (c) 2005-2006 Arch Rock Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Arch Rock Corporation nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * ARCHED ROCK OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE
 */

/**
 * Implementation of the receive path for the ChipCon CC2420 radio.
 *
 * @author Jonathan Hui <jhui@archrock.com>
 * @version $Revision: 1.3 $ $Date: 2008/06/17 07:28:24 $
 */

configuration CC2420ReceiveC {

  provides interface StdControl;
  provides interface CC2420Receive;
  provides interface Receive;
  provides interface ReceiveIndicator as PacketIndicator;
  
  //******************
  provides interface PIPReceive;
  //******************
}

implementation {
  components MainC;
  components CC2420ReceiveP;
  components CC2420PacketC;
  // PIP: making CC2420ReceiveP and CC2420TransmitP use the same Spi
  //components new CC2420SpiC() as Spi;
  components CC2420ControlC;
  
  //*************************
  components  new Alarm32khz32C();
  components SerialActiveMessageC as AM;
  components  LedsC;
  components ActiveMessageC;
  components CC2420TransmitP;  

  components LoggerC;

  //************************* 

  components HplCC2420PinsC as Pins;
  components HplCC2420InterruptsC as InterruptsC;

  components LedsC as Leds;
  CC2420ReceiveP.Leds -> Leds;

  PIPReceive = CC2420ReceiveP; /* PIP */

  StdControl = CC2420ReceiveP;
  CC2420Receive = CC2420ReceiveP;
  Receive = CC2420ReceiveP;
  PacketIndicator = CC2420ReceiveP.PacketIndicator;

  MainC.SoftwareInit -> CC2420ReceiveP;

  // PIP: making CC2420ReceiveP and CC2420TransmitP use the same Spi
  //CC2420ReceiveP.SpiResource -> Spi;
  //CC2420ReceiveP.RXFIFO -> Spi.RXFIFO;
  //CC2420ReceiveP.SFLUSHRX -> Spi.SFLUSHRX;
  //CC2420ReceiveP.SACK -> Spi.SACK;
  //CC2420ReceiveP.SpiResource -> CC2420TransmitP; // Included in PIPTransmit
  components CC2420TransmitC;
  CC2420ReceiveP.RXFIFO -> CC2420TransmitC.RXFIFO_proxy;
  CC2420ReceiveP.SFLUSHRX -> CC2420TransmitC.SFLUSHRX_proxy;
  CC2420ReceiveP.SACK -> CC2420TransmitC.SACK_proxy;
  
  CC2420ReceiveP.CSN -> Pins.CSN;
  CC2420ReceiveP.FIFO -> Pins.FIFO;
  CC2420ReceiveP.FIFOP -> Pins.FIFOP;
  CC2420ReceiveP.InterruptFIFOP -> InterruptsC.InterruptFIFOP;
  CC2420ReceiveP.CC2420Packet -> CC2420PacketC;
  CC2420ReceiveP.CC2420PacketBody -> CC2420PacketC;
  CC2420ReceiveP.PacketTimeStamp -> CC2420PacketC;
  CC2420ReceiveP.CC2420Config -> CC2420ControlC;

  //*****************************
  CC2420ReceiveP.Alarm -> Alarm32khz32C;
 
  CC2420ReceiveP.Leds -> LedsC;	
  CC2420ReceiveP.PacketTimeStamp32khz -> CC2420PacketC;
  CC2420ReceiveP.AMPacket -> ActiveMessageC;
  CC2420ReceiveP.PIPTransmit -> CC2420TransmitP;
  CC2420ReceiveP.PacketAcknowledgements->ActiveMessageC;

  components BusyWaitMicroC;
  CC2420ReceiveP.BusyWait -> BusyWaitMicroC;
  CC2420ReceiveP.Logger -> LoggerC;

  //*****************************

}
