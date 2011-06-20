#
# Check for Doxygen and enable documentation building
#
find_package( Doxygen )

OPTION(BUILD_DOCS "Build the documentation" ON)

#############################
#                           #
# Building Doxygen documents#
#                           #
#############################

ADD_CUSTOM_TARGET(docs echo "Documentation built.")
#ADD_DEPENDENCIES(docs dochtml docpdf)

IF (DOXYGEN_EXECUTABLE)
  ADD_CUSTOM_TARGET(docapi ${DOXYGEN_EXECUTABLE} "Doxyfile")
  ADD_DEPENDENCIES(docs docapi)
  CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)

  #IF (BUILD_DOCS)
  #ENDIF (BUILD_DOCS)

ENDIF (DOXYGEN_EXECUTABLE)
