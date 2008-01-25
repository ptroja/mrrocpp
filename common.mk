SYS_HEADER_DIRS = $(QNX_TARGET)/usr/include/cpp \
	$(QNX_TARGET)/usr/include/cpp/c $(QNX_TARGET)/usr/include

# Linker, compilator, and default global flags for both
CC = qcc
CXX = QCC
LD = QCC
CPPFLAGS = -I$(HOMEDIR)/include
CFLAGS =
CXXFLAGS =
LDFLAGS = ${FLAGS} -l m -l socket -l cpp

LIBDIR = $(HOMEDIR)/lib
BINDIR = $(HOMEDIR)/bin
ECP_MPDIR = $(HOMEDIR)/src/ecp_mp

AR = i386-pc-nto-qnx6.3.0-ar
ARFLAGS = cru
