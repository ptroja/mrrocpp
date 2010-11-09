#!/bin/sh

./messip_mgr &
MESSIP_PID=$!

export UI_HOST=`hostname -s`

sleep 0.5

PWD=`pwd`
DEFAULT_CONFIG="../../configs/default_file.cfg"
if [ ! -f ${DEFAULT_CONFIG} ]; then
	echo "default config file ${DEFAULT_CONFIG} missing"
	kill ${MESSIP_PID}
	exit 1
fi
CONFIG=`cat ../../configs/default_file.cfg`
CONFIG_FILE="../../${CONFIG}"
if [ ! -f ${CONFIG_FILE} ]; then
	echo "config file ${CONFIG_FILE} missing"
	kill ${MESSIP_PID}
	exit 1
fi
./configsrv ${PWD}/../ ../${CONFIG} &
CONFIGSRV_PID=$!

sleep 0.5

echo CONFIGSRV_PID=${CONFIGSRV_PID}
echo MESSIP_PID=${MESSIP_PID}

./ui

kill ${MESSIP_PID}
kill ${CONFIGSRV_PID}
