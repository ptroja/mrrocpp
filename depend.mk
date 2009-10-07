#SYS_HEADER_DIRS = $(QNX_TARGET)/usr/include/cpp $(QNX_TARGET)/usr/include/cpp/c $(QNX_TARGET)/usr/include

# Directory with dependences.
#DEPDIR = $(*D)/.deps
DEPDIR = $(dir *D).deps
# File with dependendencies.
df = $(DEPDIR)/$(*F)
# Makedepend command.
MAKEDEPEND = mkdir -p $(DEPDIR); $(GCCDEP) -M ${CPPFLAGS} -o $(DEPDIR)/$(*F).d $<; \
	cp $(df).d $(df).P; \
	sed -e 's/\#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $(df).d >> $(df).P; \
	rm -f $(df).d

# Overwritten command for *.cc compilation - also creates dependency files (*.P) in adequate directory.
%.o : %.cc
	@$(MAKEDEPEND)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

# Overwritten command for *.c compilation - also creates dependency files (*.P) in adequate directory.
%.o : %.c
	@$(MAKEDEPEND)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

## Wildcards used during dependency include.
CC_DEPENDS = ${patsubst %.cc,$(DEPDIR)/%.P,${SOURCES}} # substitute C++ sources
DEPENDS  = ${patsubst %.c,$(DEPDIR)/%.P,${CC_DEPENDS}} # substitute C sources

depdebug:
	@echo DEPENDS: $(DEPENDS)

# Include created dependency files.
-include $(DEPENDS)
