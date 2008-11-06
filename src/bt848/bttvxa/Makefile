# Makefile for bbtvx and blit (visualization)

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
	rm -f bttvx
	rm -rf *.o
