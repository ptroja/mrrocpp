#---------------------- lib catalog makefile of the MRROC++ frame system -----------------------------#
#======================================================#
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES = configsrv #y_spawn_process
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users

HOMEDIR = $(shell pwd)/../..
include $(HOMEDIR)/common.mk
#======================================================#	
.PHONY: all clean install libplayerc libplayerclient messip
#------------------------------------------------- Build rules -------------------------------------------------------#
all: messip libplayerc libplayerclient libmrrocpp.a libmrrocpp.so ${OBJECTS} ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
libplayerc:
	@$(MAKE) -C $@

libplayerclient:
	@$(MAKE) -C $@

messip:
	@$(MAKE) -C $@

#-----------------------------------------------------------------------------------------------------------------------#
MESSIP_OBJECTS = messip/logg_messip.o messip/messip_lib.o messip/messip_utils.o
MESSIP_OBJECTS: messip

LIBMRROCPP_OBJECTS = \
	ForceTrans.o \
	cfgopts.o \
	configurator.o \
	c_buffer.o r_buffer.o \
	mathtr.o \
	matrix.o \
	mis_fun.o \
	timer.o \
	srlib.o \
	playerpos_goal.o \
	$(MESSIP_OBJECTS)

libmrrocpp.a: $(LIBMRROCPP_OBJECTS)
	$(AR) $(ARFLAGS) $@ $?
	${INSTALL} $@ $(LIBDIR)

libmrrocpp.so: $(LIBMRROCPP_OBJECTS)
	$(CXX) $^ $(LDFLAGS) -shared -o $@
	${INSTALL} $@ $(LIBDIR)
#-----------------------------------------------------------------------------------------------------------------------#
y_spawn_process: y_spawn_process.o
	$(LD) $(LDFLAGS) -o $@  $^	

configsrv: configsrv.o cfgopts.o configsrv_m.o
	$(LD) $(LDFLAGS) -o $@ $^ -L$(LIBDIR) -lmessip -Wl,-rpath $(LIBDIR)
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/src -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: ${BINARIES}
	${INSTALL} ${BINARIES} $(BINDIR)
	@$(MAKE) -C messip install
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err *.a *.so ${BINARIES}
	@$(MAKE) -C libplayerc clean
	@$(MAKE) -C libplayerclient clean
	@$(MAKE) -C messip clean
#-----------------------------------------------------------------------------------------------------------------------#

# include $(HOMEDIR)/depend.mk
