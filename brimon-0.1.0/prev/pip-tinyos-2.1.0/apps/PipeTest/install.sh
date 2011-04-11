if [ $# -ne 2 ]
then
	echo "Usage: $0 <minnodeid> <maxnodeid>";
	exit 1;
fi

minnodeid=$1;
maxnodeid=$2;


for x in `seq $minnodeid $maxnodeid`
do
	ttynum=$(($maxnodeid - $x))
	nodeid=`expr $minnodeid + $maxnodeid - $x`;
	if [ $nodeid -eq 1 ]
	    then
	    sleep 3
	fi


	echo "nodeid = ${nodeid}, tty = /dev/ttyUSB${ttynum}:telosb"
	make tmote reinstall,$nodeid bsl,/dev/ttyUSB${ttynum} &
done

