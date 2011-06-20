# Locate COMEDI install directory

# This module defines
# COMEDI_INSTALL where to find include, lib, bin, etc.
# COMEDILIB_FOUND, is set to true

#INCLUDE (${PROJ_SOURCE_DIR}/config/FindPkgConfig.cmake)

# GNU/Linux detection of Comedi lib uses pkgconfig or plain header search.
IF ( NOT QNXNTO)

#  IF ( CMAKE_PKGCONFIG_EXECUTABLE )
#      SET(ENV{PKG_CONFIG_PATH} "${COMEDI_INSTALL}/lib/pkgconfig/")
#      MESSAGE( "Looking for comedilib headers in ${COMEDI_INSTALL}/include ")
#      PKGCONFIG( "comedilib >= 0.7.0" COMEDILIB_FOUND COMEDI_INCLUDE_DIRS COMEDI_DEFINES COMEDI_LINK_DIRS COMEDI_LIBS )
#  ENDIF( CMAKE_PKGCONFIG_EXECUTABLE )

  IF( NOT COMEDI_INSTALL )
    SET(COMEDI_INSTALL "/usr")
  ENDIF( NOT COMEDI_INSTALL )

  IF ( NOT COMEDILIB_FOUND )
    MESSAGE(STATUS "Manually looking for comedilib headers in ${COMEDI_INSTALL}/include")
    FIND_FILE(COMEDILIB_FOUND "comedilib.h" ${COMEDI_INSTALL}/include)
    SET(COMEDI_INCLUDE_DIRS "${COMEDI_INSTALL}/include")
    SET(COMEDI_LINK_DIRS "${COMEDI_INSTALL}/lib")
    SET(COMEDI_LIBS "comedi")
  ENDIF ( NOT COMEDILIB_FOUND )

  IF( COMEDILIB_FOUND )
    FIND_LIBRARY(COMEDI_LIBRARY NAMES comedilib comedi HINTS "${COMEDI_LINK_DIRS}") 
        MESSAGE(STATUS "   Comedi Lib found.")
        MESSAGE(STATUS "   Includes in: '${COMEDI_INCLUDE_DIRS}'")
        MESSAGE(STATUS "   Libraries in: '${COMEDI_LINK_DIRS}'")
        MESSAGE(STATUS "   Libraries: '${COMEDI_LIBS}'")
        MESSAGE(STATUS "   Defines: '${COMEDI_DEFINES}'")
        MESSAGE(STATUS "   Link Library: '${COMEDI_LIBRARY}'")
    SET(COMEDI_FOUND TRUE)
  ENDIF( COMEDILIB_FOUND )
ENDIF ( NOT QNXNTO )

