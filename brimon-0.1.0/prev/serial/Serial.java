/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

import java.io.IOException;

import net.tinyos.message.*;
import net.tinyos.packet.*;
import net.tinyos.util.*;

public class Serial implements MessageListener {

  private MoteIF moteIF;
  
  public Serial(MoteIF moteIF) {
    this.moteIF = moteIF;
    this.moteIF.registerListener(new SerialMsg(), this);
  }

  public void sendPackets() {
    int counter = 0;
    SerialMsg payload = new SerialMsg();
    
    try {
      while (true) {
	System.out.println("Sending packet " + counter);
	payload.set_counter(counter);
	moteIF.send(0, payload);
	counter++;
	try {Thread.sleep(1000);}
	catch (InterruptedException exception) {}
      }
    }
    catch (IOException exception) {
      System.err.println("Exception thrown when sending packets. Exiting.");
      System.err.println(exception);
    }
  }

  public void messageReceived(int to, Message message) {
    SerialMsg msg = (SerialMsg)message;
    System.out.println("Received packet sequence number " + msg.get_counter());
  }
  
  private static void usage() {
    System.err.println("usage: Serial [-comm <source>]");
  }
  
  public static void main(String[] args) throws Exception {
    String source = null;
    if (args.length == 2) {
      if (!args[0].equals("-comm")) {
	usage();
	System.exit(1);
      }
      source = args[1];
    }
    else if (args.length != 0) {
      usage();
      System.exit(1);
    }
    
    PhoenixSource phoenix;
    
    if (source == null) {
      phoenix = BuildSource.makePhoenix(PrintStreamMessenger.err);
    }
    else {
      phoenix = BuildSource.makePhoenix(source, PrintStreamMessenger.err);
    }

    MoteIF mif = new MoteIF(phoenix);
    Serial serial = new Serial(mif);
    serial.sendPackets();
  }


}
