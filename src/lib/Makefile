#---------------------- lib catalog makefile of the MRROC++ frame system -----------------------------#
#======================================================#
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
ifeq ($(BUILD_TARGET), linux)
BINARIES = configsrv #y_spawn_process
endif
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users

HOMEDIR = $(shell pwd)/../..
include $(HOMEDIR)/common.mk
#======================================================#	
.PHONY: all clean install libplayerc libplayerclient
#------------------------------------------------- Build rules -------------------------------------------------------#
all: libplayerc libplayerclient libmrrocpp.a libmrrocpp.so ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
libplayerc:
	@$(MAKE) -C $@

libplayerclient:
	@$(MAKE) -C $@

#-----------------------------------------------------------------------------------------------------------------------#


LIBMRROCPP_OBJECTS = \
	mrmath/ft_v_tr.o \
	mrmath/ForceTrans.o \
	mrmath/homog_matrix.o \
	mrmath/mathtr.o \
	mrmath/k_vector.o \
	mrmath/ft_v_vector.o \
	mrmath/jacobian_matrix.o \
	cfgopts.o \
	configurator.o \
	c_buffer.o r_buffer.o \
	matrix.o \
	mis_fun.o \
	timer.o \
	srlib.o \
	playerpos_goal.o \
	datastr.o 

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
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err *.a *.so ${BINARIES} ./.deps/*.P ./mrmath/*.o ./mrmath/*.err
	@$(MAKE) -C libplayerc clean
	@$(MAKE) -C libplayerclient clean
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk
