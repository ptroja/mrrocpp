#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES = 	ecp_test_conv ecp_kon ecp_legobrick_conv ecp_visconv ecp_visconvsin
				
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ecp_r_conv.o ../common/libecp.a $(LIBDIR)/libmrrocpp.a

#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#---------------------------------Special object build rules (user made)-------------------------------------#

ecp_test_conv: task/ecp_t_test.o generator/ecp_g_test.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_kon: task/ecp_t_kon.o ../common/generator/ecp_g_jarosz.o ../common/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_legobrick_conv: task/ecp_t_legobrick_conv.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_jarosz.o ../common/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_visconv: task/ecp_t_visconv.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_jarosz.o ../common/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_visconvsin: task/ecp_t_visconvsin.o generator/ecp_g_visconv.o ../common/generator/ecp_g_jarosz.o ../common/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: ${BINARIES}
	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P ./task/*.o ./task/*.err ./generator/*.o ./generator/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk
