#///////////////////////////////////////////////////////////////////////////////
#//
#//                              TECHNICAL SPECIFICATION
#//
#///////////////////////////////////////////////////////////////////////////////
#// 
#// $URL: $
#//
#///////////////////////////////////////////////////////////////////////////////
#// Revision  : $LastChangedRevision: $
#// Project   : MRROC++
#///////////////////////////////////////////////////////////////////////////////
#// Created   : 2008-01-24
#// Developer : ptroja
#// Modified  : $LastChangedDate:  $
#// By        : $LastChangedBy:  $
#///////////////////////////////////////////////////////////////////////////////
#// File description:
#// 
#//   File with common Makefile settings
#// 
#///////////////////////////////////////////////////////////////////////////////


# Linker, compilator, and default global flags for both
#VERSION=-V3.3.5,gcc_ntox86
#VERSION=-V4.2.1,gcc_ntox86
CC = qcc ${VERSION}
CXX = QCC ${VERSION}
LD = QCC ${VERSION}
CPPFLAGS = -I$(HOMEDIR)/include
DEBUG= -g
#OPTIM = -O2
#WARN = -w9
FLAGS = ${DEBUG} ${OPTIM} ${WARN}
CFLAGS = ${FLAGS}
CXXFLAGS = ${FLAGS}
LDFLAGS = ${DEBUG} -l m -l socket -l cpp -lang-c++

LIBDIR = $(HOMEDIR)/lib
BINDIR = $(HOMEDIR)/bin
ECP_MPDIR = $(HOMEDIR)/src/ecp_mp

AR = ntox86-ar
#AR = ntox86-ar-2.17
ARFLAGS = cru

INSTALL = ln -f
#INSTALL = cp
