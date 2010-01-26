#///////////////////////////////////////////////////////////////////////////////
# /** @file     common.mk
#  *
#  *  File with common Makefile settings
#  *
#  *  @author   ptroja
#  *  @date     2008-01-24
#  *  $URL$
#  *  $LastChangedRevision$
#  *  $LastChangedDate$  
#  *  $LastChangedBy$
#  *  
#  */
#////////////////////////////////////////////////////////////////////////////////

DEBUG=-g -mtune=i686
#OPTIM = -O2 -mtune=i686
#WARN = -w0
FLAGS=-fPIC ${DEBUG} ${OPTIM} ${WARN}
CFLAGS=${FLAGS}
CXXFLAGS=${FLAGS}

# Linker, compilator, and default global flags for both
ifeq ($(BUILD_TARGET), linux)
  # for gcc toolchain:
  LD=g++
  CC=gcc
  CXX=g++
  GCCDEP=gcc
  # for Intel Compiler toolchain:
  #LD=icpc
  #CC=icc
  #CXX=icpc
  #AR=xiar
  #GCCDEP=icpc
  # for SunStudio SunCC toolchain: this is not ready yet since sunCC does
  # not support variable lenght arrays (C99 extension imported to g++/QCC/icpc)
  #CXX=sunCC -mt
  #CC=suncc -mt
  #LD=suncc/sunCC
  #GCCDEP=gcc
  # other flags
  LDFLAGS=-lrt `pkg-config --libs libxml-2.0` -lboost_thread-mt -lmessip
  CPPFLAGS=-I$(HOMEDIR)/src `pkg-config --cflags libxml-2.0` -DUSE_MESSIP_SRR -Wall -pipe #-Werror
  # uncomment the following for Solaris OS
  #LDFLAGS+=-L/export/home/ptroja/boost_1_39_0/stage/lib -lboost_thread-gcc34-mt \
  #	-lsocket -lnsl -lpthread
  #CPPFLAGS+=-D_POSIX_PTHREAD_SEMANTICS -pthreads -I/export/home/ptroja/boost_1_39_0
  BINDIR=$(HOMEDIR)/bin.linux
  LIBDIR=$(HOMEDIR)/lib.linux
else
  VERSION=-V4.3.3,gcc_ntox86
  RPATHV=-Wl,-rpath /usr/pkg/lib
  CC=qcc ${VERSION}
  CXX=QCC ${VERSION}
  LD=QCC ${VERSION}
  LDFLAGS=${DEBUG} -lm -lsocket -lcpp -lang-c++ $(RPATHV) -L$(QNX_TARGET)/mrlib/lib -lxml2 -liconv -lboost_thread-mt
  CPPFLAGS=-I$(HOMEDIR)/src -I$(QNX_TARGET)/mrlib/include
  AR=ntox86-ar
  BINDIR=$(HOMEDIR)/bin
  LIBDIR=$(HOMEDIR)/lib
  GCCDEP=ntox86-gcc
endif

UI_DIR = $(HOMEDIR)/src/ui/src/gcc_ntox86
ECP_DIR = $(HOMEDIR)/src/ecp
ECP_MPDIR=$(HOMEDIR)/src/ecp_mp
KIN_DIR=$(HOMEDIR)/src/kinematics

#ifeq (,$(findstring "host/qnx/x86",$(QNX_HOST)))
ifneq (,$(findstring host/qnx6/x86,$(QNX_HOST)))
REPOSITORY_PATH=
else
REPOSITORY_PATH=$(QNX_TARGET)
endif

ARFLAGS=cru
#AR=ntox86-ar-2.17


ifeq ($(OS),Windows_NT)
	INSTALL=cp
else
	INSTALL=ln -f
endif



USEFILE=

#test zmiany repozytorium
