#!/bin/sh
# cross-compilation for Linux UI

#QNX binaries
make TARGET=qnx
make TARGET=qnx install

make clean

#Linux UI libraries + messip binaries
make TARGET=linux
make TARGET=linux install

#temporary solution - above doesn't seem to compile properly the UI libraries (though compile properly messip binaries)
cd ui/src/gcc_ntox86
make BUILD_TARGET=linux -k 2>/dev/null
cd ../../..


#Linux UI binaries
make TARGET=linux_gtk
make TARGET=linux_gtk install