#! /usr/bin/env bash


# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# Written (W) 2011 Abhinav Maurya
# Copyright (C) 2011 Abhinav Maurya


# Here we run the following:-
# the setup script to setup environment variables
# the make telosb command to build the application
# the make command to burn the binary to mote
# set the five variables given below as per the path of these repositories on your system

APPROOT="/home/ahmaurya/Dropbox/Courses/CS691/brimon/brimon-0.1.0/application"
TOSROOT="/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1"
TOSDIR="$TOSROOT/tos"
CLASSPATH="$CLASSPATH:/home/ahmaurya/bin/java:$TOSROOT/support/sdk/java/tinyos.jar:."
MAKERULES="$TOSROOT/support/make/Makerules"

ifburn=false
ifclean=false
ifdocs=false
ifid=false
mote=""

while getopts :bcdin: opt
do
    case "$opt" in
      b)  ifburn=true;;
      c)  ifclean=true;;
      d)  ifdocs=true;;
      i)  ifid=true;;
      n)  mote="$OPTARG";;
      ?)  echo;
	  echo "Usage: $0 [-b] [-c] [-d] [-i] [-n device_name]";
	  echo "        b: burns application to all connected motes";
	  echo "        c: cleans the build and doc directories";
	  echo "        d: creates application documentation";
	  echo "        i: burns application to all connected motes; allows specifying TOS_NODE_ID interactively";
	  echo "        n: burns application to a single mote specified by device name; default=/dev/ttyUSB0";
	  echo;
	  exit 1;;
    esac
done
shift `expr $OPTIND - 1`

echo
echo "***Setting up environment variables***"
echo "----------------------------------------"
echo "Setting up for TinyOS 2.1.1"
export APPROOT
export TOSROOT
export TOSDIR
export CLASSPATH
export MAKERULES
cd $APPROOT
echo

#Build the nesC documentation and exit
if $ifdocs == true;
then
	echo "***Building the nesC documentation***"
	echo "----------------------------------------"
	make telosb docs
	echo
	exit 0
fi

#Clean any previous nesC application builds and exit
if $ifclean == true;
then
	echo "***Cleaning any previous nesC application builds***"
	echo "----------------------------------------"
	echo "Cleaning previous nesC application build..."
	make -s -f "$APPROOT/Makefile" clean
	echo "Done!"
	echo "Cleaning previous nesC application documentation..."
	rm -rf $APPROOT/doc
	echo "Done!"
	echo
	exit 0
fi

#Burn the nesC application to a specific mote connected to system
#Mote ID must be specified using the -n command-line option
if [ -n "$mote" ]
then
	echo "***Burning the nesC application***"
	echo "----------------------------------------"
	echo
	echo "Burning application to mote $mote"
	echo "----------------------------------------"
	chmod 666 $mote
	make -s telosb install bsl,$mote
	echo
	exit 0
fi

#Burn the nesC application to all motes connected to system
if $ifburn == true;
then
	echo "***Burning the nesC application***"
	echo "----------------------------------------"
	for mote in `motelist | awk 'NR<=2{next} {print $2}'`
	do
		echo
		echo "Burning application to mote $mote"
		echo "----------------------------------------"
		chmod 666 $mote
		make -s telosb install bsl,$mote
	done
	echo
	exit 0
fi

#Burn the nesC application to all motes connected to system
#Each mote is burnt with a different ID
#Mote IDs can be specified interactively via commandline
if $ifid == true;
then
	echo "***Burning the nesC application***"
	echo "----------------------------------------"
	for mote in `motelist | awk 'NR<=2{next} {print $2}'`
	do
		echo
		echo "Burning application to mote $mote"
		echo "----------------------------------------"
		echo -n "Enter the node ID for node $mote: ";
		read nodeid
		chmod 666 $mote
		make -s telosb install,$nodeid bsl,$mote
	done
	echo
	exit 0
fi

#Build the nesC application
#Default operation in case no clean, documentation or burn command-line options
echo "***Building the nesC application***"
echo "----------------------------------------"
make -s telosb
echo
