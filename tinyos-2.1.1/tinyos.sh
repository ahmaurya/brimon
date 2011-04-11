#! /usr/bin/env bash
# Here we setup the environment
# variables needed by the tinyos 
# make system

echo "Setting up for TinyOS 2.1.1"
export TOSROOT=
export TOSDIR=
export MAKERULES=

TOSROOT="/home/ahmaurya/Dropbox/Courses/CS691/brimon/src/tinyos-2.1.1"
TOSDIR="$TOSROOT/tos"
CLASSPATH=$CLASSPATH:/home/ahmaurya/bin/java:$TOSROOT/support/sdk/java/tinyos.jar:.
MAKERULES="$TOSROOT/support/make/Makerules"

export TOSROOT
export TOSDIR
export CLASSPATH
export MAKERULES

