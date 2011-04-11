Readme for BriMon source code:-
----------------------------------------
	
1. Place and edit the nesC application code in brimon-x.y.z folder of the project.

2. Change the APPROOT in the build.sh file in project folder. Set APPROOT to <PATH_TO_APPLICATION>
 
3. Change the TOSROOT in the build.sh file in project folder. Set TOSROOT to <PATH_TO_TINYOS>/tinyos-2.1.1

4. Plug in the TelosB mote if you wish to burn the program to the mote. Verify that it has been detected by using the command motelist.

5. Run build.sh script in the project folder in superuser mode. It will set up the required environment variables, build and burn the nesC application on your mote.

6. build.sh can be used with the following command-line options:-

	Usage: ./build.sh [-b] [-c] [-d] [-i] [-n device_name]
		b: burns application to all connected motes
		c: cleans the build and doc directories
		d: creates application documentation
		i: burns application to all connected motes; allows specifying TOS_NODE_ID interactively
		n: burns application to a single mote specified by device name; default=/dev/ttyUSB0

7. As a more convenient alternative to 5 and 6, you may use the make command which handles all dependencies. Make command may be used with the following options:-

	Usage: make [ all | build | burn | burnb | burni | burnn | clean | docs ]
	
----------------------------------------
