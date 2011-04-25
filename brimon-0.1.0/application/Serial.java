/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Abhinav Maurya
 * Copyright (C) 2011 Abhinav Maurya
 */

import java.io.*;

import net.tinyos.message.*;
import net.tinyos.packet.*;
import net.tinyos.util.*;

/**
 * The class serial is the control class that coupled with the
 * generated SerialMsg class forms the code that interacts with
 * the mote on the serial port. This is the main class to be
 * executed on the commandline using the following command:
 * <br/>
 * java Serial -comm serial@<name-of-device>:baudrate
 * <br/>
 */

public class Serial implements MessageListener {

	/** abstraction for interacting with the mote on the serial port */
	private MoteIF moteIF;

	/** keeps track of the fraction of log received */
	private int counter = 0;

	/** used to recieve command from the commandline and send it to the mote */
	private int commandType;

	/** output file for the received log on the system */
	private static String SENSE_FILE = "sense.dat";

	/** various command codes that concur with those found in Command.h utilized by the mote */
	private static final int
		BEACON = 0,
		ROUTING = 1,
		TIMESYNC_START = 2,
		TIMESYNC_STOP = 3,
		SENSE = 4,
		COLLECT = 5,
		TRANSFER = 6,
		ROUTING_COMPLETE = 7,
		TIMESYNC_START_COMPLETE = 8,
		TIMESYNC_STOP_COMPLETE = 9,
		SENSE_COMPLETE = 10,
		COLLECT_COMPLETE = 11,
		TRANSFER_COMPLETE = 12,
		DIFF_COMMAND_COMPLETE = 6;

	/** reading a file from disk */
	FileInputStream fis;

	/** writing out a file to disk */
	FileOutputStream fos;

	/** consstructor using a given MoteIF object */
	public Serial(MoteIF moteIF) {
		this.moteIF = moteIF;
		this.moteIF.registerListener(new SerialMsg(), this);
	}

	/** code to send packets to mote based on commands received on the commandline */
	public void sendPackets() {
		SerialMsg payload = new SerialMsg();
		try {
			try {
				Thread.sleep(1000);
			}
			catch (InterruptedException exception) {}
			System.out.println("\n\nFollowing commands are available:\n\t1: ROUTING\n\t2: TIMESYNC_START\n\t3: TIMESYNC_STOP\n\t4: SENSE\n\t5: COLLECT\n\t6: TRANSFER\n");
			System.out.print("Enter a command: ");
			InputStreamReader isr = new InputStreamReader(System.in);
			BufferedReader inData = new BufferedReader(isr);
			commandType = (new Integer(inData.readLine())).intValue();

			//System.out.println("Sending packet " + counter + " with command type " + commandType);
			payload.set_type(commandType);
			//payload.set_counter(counter);

			//send command to mote
			moteIF.send(0, payload);
			counter++;
		}
		//handle any exceptions that might occur
		catch (Exception exception) {
		  //System.err.println("Exception thrown when sending packets. Exiting.");
		  //System.err.println(exception);
		  System.exit(1);
		}
	}

	/** called when any packets are received from the serial port on which this program is listening */
	public void messageReceived(int to, Message message) {
		SerialMsg msg = (SerialMsg)message;
		//process the packet as per the type field of the packet recived on the serial port
		if(msg.get_type()==commandType || msg.get_type()==commandType+DIFF_COMMAND_COMPLETE)
		switch(msg.get_type())
		{
			case ROUTING:
			case TIMESYNC_START:
			case TIMESYNC_STOP:
			case SENSE:
			case COLLECT:
				break;
			case TRANSFER:
				try {
					fos = new FileOutputStream(SENSE_FILE, true);
					short[] temp = msg.get_payload();
					for(int i=0;i<temp.length;i++) {
						fos.write((byte)temp[i]);
						//System.out.print("\t"+temp[i]);
					}
					fos.flush();
					fos.close();
				}
				catch(Exception e) {System.out.println("Routing Tree (Child --> Parent):");}
				break;
			case ROUTING_COMPLETE:
				//print routing tree
				System.out.println("Routing Tree (Child --> Parent):");
				for(int i=0; i<msg.get_counter();i++)
					System.out.println("\t"+(i+1)+" --> "+msg.getElement_payload(i));
			case TIMESYNC_START_COMPLETE:
			case TIMESYNC_STOP_COMPLETE:
			case SENSE_COMPLETE:
			case COLLECT_COMPLETE:
			case TRANSFER_COMPLETE:
				//signal completion of command execution
				System.out.println("Finished executing command " + (msg.get_type()-DIFF_COMMAND_COMPLETE));
				sendPackets();
				break;
		}
	}

	/** display correct usage of invoking the program */
	private static void usage() {
		System.err.println("usage: Serial [-comm <source>]");
	}

	/** main class that checks for correct connectivity to serial port
	 * and if the usage is correct, starts the interactive command execution system
	 */
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

		//creating an object abstraction for automatic reception and dispatching of packets on serial port
		PhoenixSource phoenix;

		if (source == null) {
			phoenix = BuildSource.makePhoenix(PrintStreamMessenger.err);
		}
		else {
			phoenix = BuildSource.makePhoenix(source, PrintStreamMessenger.err);
		}

		// the mote abstraction builds on the PhoenixSource object
		MoteIF mif = new MoteIF(phoenix);
		Serial serial = new Serial(mif);
		serial.sendPackets();
	}
}
