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
	
