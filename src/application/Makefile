# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

SOURCEDIRS = time wii_teach wii_velocity wii_ellipse haptic sk servovision calibration teach playerjoy smooth-tester
INSTALLDIRS = ${SOURCEDIRS}
CLEANDIRS = ${SOURCEDIRS}
#======================================================#
.PHONY: all clean install $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#

all: $(SOURCEDIRS)

wii_velocity: wii_teach
wii_ellipse: wii_teach

$(SOURCEDIRS):
	@$(MAKE) -C $@

#	for i in ${SOURCEDIRS} ; do \
#		($(MAKE) -C $$i) ; \
#	done
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
clean:
	@for i in ${CLEANDIRS} ; do ($(MAKE) -C $$i clean) ; done
#-----------------------------------------------------------------------------------------------------------------------#
install:
	@for i in ${INSTALLDIRS} ; do ($(MAKE) -C $$i install) ; done
#======================================================#	
