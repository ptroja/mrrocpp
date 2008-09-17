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
#///////////////////////////////////////////////////////////////////////////////

# Linker, compilator, and default global flags for both
#VERSION=-V2.95.3,gcc_ntox86
VERSION=-V3.3.5,gcc_ntox86
#VERSION=-V4.2.1,gcc_ntox86
CC=qcc ${VERSION}
CXX=QCC ${VERSION}
CXXFLAGS=${FLAGS}
LD=QCC ${VERSION}
LDFLAGS=${DEBUG} -lm -lsocket -lcpp -lang-c++ -L$(HOMEDIR)/lib/libxml -lxml2 -lz -liconv
CPPFLAGS=-I$(HOMEDIR)/include
#DEBUG=-g
#OPTIM = -O2
#WARN = -w9
FLAGS=${DEBUG} ${OPTIM} ${WARN}
CFLAGS=${FLAGS}

LIBDIR=$(HOMEDIR)/lib
BINDIR=$(HOMEDIR)/bin
ECP_MPDIR=$(HOMEDIR)/src/ecp_mp

#ifeq (,$(findstring "host/qnx/x86",$(QNX_HOST)))
ifneq (,$(findstring host/qnx6/x86,$(QNX_HOST)))
REPOSITORY_PATH=
else
REPOSITORY_PATH=$(QNX_TARGET)
endif

AR=ntox86-ar
ARFLAGS=cru
#AR=ntox86-ar-2.17

INSTALL=ln -f
#INSTALL = cp
#===== USEFILE - the file containing the usage message for the application. 
USEFILE=

#test zmiany repozytorium
