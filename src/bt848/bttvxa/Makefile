# Makefile for bbtvx and blit (visualization)



HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

CC=qcc
CFLAGS= -ggdb3
LDFLAGS=
SRC=bttvx.c
SRC2=blit.c
OBJS=bttvx.o
OBJS2=blit.o
DIST=bttvx_0.2.2


all: $(OBJS) bttvx 

bttvx: $(OBJS)
	   $(CC) $(LDFLAGS) $(OBJS) -o $@


clean:
	@rm -f *.o *.err bttvx
	
install: bttvx
	${INSTALL} bttvx $(BINDIR)

