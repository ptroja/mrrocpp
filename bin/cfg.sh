#!/bin/sh

./messip_mgr &
MESSIP_PID=$!

export UI_HOST=`hostname` 

sleep 1

PWD=`pwd`
CONFIG=`cat ../configs/default_file.cfg`
./configsrv `hostname` `dirname ${PWD}`/ ${CONFIG} &
CONFIGSRV_PID=$!

sleep 1

echo CONFIGSRV_PID=${CONFIGSRV_PID}
echo MESSIP_PID=${MESSIP_PID}

./ui

kill ${MESSIP_PID}
kill ${CONFIGSRV_PID}
