# Linker, compilator, and default global flags for both
CC = qcc
CXX = QCC
LD = QCC
CPPFLAGS = -I$(HOMEDIR)/include
#DEBUG = -g
#OPTIM = -O2
#WARN = -w9
FLAGS = ${DEBUG} ${OPTIM} ${WARN}
CFLAGS = ${FLAGS}
CXXFLAGS = ${FLAGS}
LDFLAGS = ${DEBUG} -l m -l socket -l cpp

LIBDIR = $(HOMEDIR)/lib
BINDIR = $(HOMEDIR)/bin
ECP_MPDIR = $(HOMEDIR)/src/ecp_mp

AR = i386-pc-nto-qnx6.3.0-ar
ARFLAGS = cru

#INSTALL = ln -f
INSTALL = cp
