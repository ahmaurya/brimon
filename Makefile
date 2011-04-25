# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# Written (W) 2011 Abhinav Maurya
# Copyright (C) 2011 Abhinav Maurya

all: build docs

build:
	sh build.sh

docs:
	sh build.sh -d

clean:
	sh build.sh -c

burn: burnb

burnb: build
	sh build.sh -b
	
burni: build
	sh build.sh -i
	
burnn: build
	sh build.sh -n /dev/ttyUSB0
	
