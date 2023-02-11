#!/bin/bash

[ $# -lt 3 ] && {
	echo "usage $0 <barcode> <file name> <dtb> [<tftp directory>]"
	exit 1
}

BASEDIR=/u/cs452/public/tftp/RPi

[ -d $BASEDIR ] || {
	echo "cannot find tftp directory: " $BASEDIR
	exit 1
}

case "$1" in
	CS017540|CS017541|CS017542|CS017543|CS017544|MartinTest) DESTDIR=$BASEDIR/$1;;
	*) echo "invalid barcode: " $1; exit 1;;
esac

[ -r $2 ] || {
	echo "cannot find boot file: " $2
	exit 1
}

function cleanup() {
	/bin/chmod -R ugo+rw $DESTDIR
	/bin/rm -rf $DESTDIR
	echo "removed $DESTDIR"
	exit 1
}

umask 0000
/bin/rm -rf $DESTDIR
trap cleanup 1 2 3 4 5 6 7 8 10 11 12 13 14 15 ERR
/bin/cp -a $BASEDIR/boot/ $DESTDIR
/bin/chmod ugo+rwx $DESTDIR
/bin/cp -f $2 $DESTDIR/kernel8.img
/bin/cp -f $3 $DESTDIR/bcm2711-rpi-4-b.dtb
/bin/chmod ugo+rwx $DESTDIR
/bin/chmod -R ugo+rw $DESTDIR

echo "finished setting up $DESTDIR"
exit 0
