#!/bin/sh


cd /root/


BASH_PATH=/usr/pkg/bin/bash
SH_PATH=/bin/sh
PASSWD_TMP=/root/passwd.old
PP=`grep $BASH_PATH /etc/passwd`
if [ "$PP" ]; then
	echo "Replacing $BASH_PATH to $SH_PATH in /etc/passwd (temp file $PASSWD_TMP)"
	cp /etc/passwd "$PASSWD_TMP"
	sed "s!$BASH_PATH!$SH_PATH!" "$PASSWD_TMP" > /etc/passwd
fi


echo "Removing setupqnx.py"
rm -f /root/setupqnx.py

PACKAGES=`pkg_info | cut -d ' ' -f 1`

for p in $PACKAGES; do

	echo "Removing $p"
	pkg_delete -f $p

done


ftp http://github.com/ptroja/mrrocpp/raw/master/utils/setupqnx.py
python setupqnx.py

echo "Koniec"
