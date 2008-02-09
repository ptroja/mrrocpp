SYS_HEADER_DIRS = $(QNX_TARGET)/usr/include/cpp \
	$(QNX_TARGET)/usr/include/cpp/c $(QNX_TARGET)/usr/include

depend:
	@touch .depend
	@makedepend	-f.depend -I$(HOMEDIR)/include  \
							${patsubst %,-I%,${SYS_HEADER_DIRS}} \
							${SOURCES} 2> /dev/null

-include .depend
