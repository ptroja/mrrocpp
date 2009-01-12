#!/bin/sh

./messip_mgr &
MESSIP_PID=$!

export UI_HOST=`hostname` 

sleep 1

PWD=`pwd`
DEFAULT_CONFIG="../configs/default_file.cfg"
if [ ! -f ${DEFAULT_CONFIG} ]; then
	echo "default config file ${DEFAULT_CONFIG} missing"
	kill ${MESSIP_PID}
	exit 1
fi
CONFIG=`cat ../configs/default_file.cfg`
CONFIG_FILE="../configs/${CONFIG}"
if [ ! -f ${CONFIG_FILE} ]; then
	echo "config file ${CONFIG_FILE} missing"
	kill ${MESSIP_PID}
	exit 1
fi
./configsrv `hostname` `dirname ${PWD}`/ ${CONFIG} &
CONFIGSRV_PID=$!

sleep 1

echo CONFIGSRV_PID=${CONFIGSRV_PID}
echo MESSIP_PID=${MESSIP_PID}

./ui.gtk

kill ${MESSIP_PID}
kill ${CONFIGSRV_PID}
