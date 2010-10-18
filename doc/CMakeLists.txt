#
# Check for Doxygen and enable documentation building
#
find_package( Doxygen )
IF ( DOXYGEN_EXECUTABLE )
  MESSAGE( STATUS "Found Doxygen -- API documentation can be built" )
ELSE ( DOXYGEN_EXECUTABLE )
  MESSAGE( STATUS "Doxygen not found -- unable to build documentation" )
ENDIF ( DOXYGEN_EXECUTABLE )

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
ENDIF (DOXYGEN_EXECUTABLE)
