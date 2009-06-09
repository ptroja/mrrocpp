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

DEBUG=-g
#OPTIM = -O2
#WARN = -w9
FLAGS=${DEBUG} ${OPTIM} ${WARN}
CFLAGS=${FLAGS}

# Linker, compilator, and default global flags for both
ifeq ($(BUILD_TARGET), linux)
  LD=g++
  CC=gcc
  CXX=g++
  LDFLAGS=-lrt `pkg-config --libs libxml-2.0`
  CPPFLAGS=-Wall -I$(HOMEDIR)/include `pkg-config --cflags libxml-2.0` -DUSE_MESSIP_SRR
  BINDIR=$(HOMEDIR)/bin.linux
else
  VERSION=-V4.3.3,gcc_ntox86
  RPATHV=-Wl,-rpath /usr/pkg/lib
  CC=qcc ${VERSION}
  CXX=QCC ${VERSION}
  CXXFLAGS=${FLAGS}
  LD=QCC ${VERSION}
  LDFLAGS=${DEBUG} -lm -lsocket -lcpp -lang-c++ $(RPATHV) -L$(QNX_TARGET)/mrlib/lib -lxml2 -liconv
  CPPFLAGS=-I$(HOMEDIR)/include -I$(QNX_TARGET)/mrlib/include
  AR=ntox86-ar
  BINDIR=$(HOMEDIR)/bin
endif

LIBDIR=$(HOMEDIR)/lib
UI_DIR = $(HOMEDIR)/src/ui/src/gcc_ntox86
ECP_DIR = $(HOMEDIR)/src/ecp
ECP_MPDIR=$(HOMEDIR)/src/ecp_mp

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



#===== USEFILE - the file containing the usage message for the application. 
USEFILE=

#test zmiany repozytorium
