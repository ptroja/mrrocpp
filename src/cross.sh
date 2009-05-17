#!/bin/sh
# cross-compilation for Linux UI

#QNX binaries
#make TARGET=qnx
#make TARGET=qnx install

make clean

#Linux UI libraries + messip binaries
make TARGET=linux
make TARGET=linux install

#Linux UI binaries
make TARGET=linux_gtk
make TARGET=linux_gtk install