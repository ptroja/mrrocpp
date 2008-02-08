#---------------------- lib catalog makefile of the MRROC++ frame system -----------------------------#
#======================================================#
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES = configsrv y_spawn_process
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk
#======================================================#	
.PHONY: all clean install depend libplayerc messip
#------------------------------------------------- Build rules -------------------------------------------------------#
all: messip libplayerc libmrrocpp.a ${OBJECTS} ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
libplayerc:
	@$(MAKE) -C $@

messip:
	@$(MAKE) -C $@

#-----------------------------------------------------------------------------------------------------------------------#
MESSIP_OBJECTS = messip/logg_messip.o messip/messip_lib.o messip/messip_utils.o
MESSIP_OBJECTS: messip

libmrrocpp.a: \
	ForceTrans.o \
	cfgopts.o \
	configurator.o \
	cr_buf.o \
	mathtr.o \
	matrix.o \
	mis_fun.o \
	mp_timer.o \
	nurbs.o \
	srlib.o \
	$(MESSIP_OBJECTS)
	$(AR) $(ARFLAGS) $@ $?
	cp $@ $(LIBDIR)

#-----------------------------------------------------------------------------------------------------------------------#
y_spawn_process: y_spawn_process.o
	$(LD) $(LDFLAGS) -o $@  $^	

configsrv: configsrv.o cfgopts.o configsrv_m.o
	$(LD) $(LDFLAGS) -o $@ $^ -L$(LIBDIR) -lmessip
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: ${BINARIES}
	cp ${BINARIES} $(BINDIR)
	@$(MAKE) -C messip install
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err *.a ${BINARIES}
	@$(MAKE) -C libplayerc clean
	@$(MAKE) -C messip clean
#-----------------------------------------------------------------------------------------------------------------------#
#DEPDIR = .deps
#df = $(DEPDIR)/$(*F)
#
#export MAKEDEPEND = i386-pc-nto-qnx6.3.0-gcc -M $(CPPFLAGS) $(CXXFLAGS) -o $(DEPDIR)/$*.d $<
#
#%.o : %.cc
#	@$(MAKEDEPEND); \
#	cp $(df).d $(df).P; \
#	  sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
#	  -e '/^$$/ d' -e 's/$$/ :/' < $(df).d >> $(df).P; \
#	rm -f $(df).d
#	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $@ $<
#
#-include $(SRCS:%.c=$(DEPDIR)/%.P)

include $(HOMEDIR)/depend.mk
