# The name of our project is "MRROCPP". CMakeLists files in this project can
# refer to the root source directory of the project as ${MRROCPP_SOURCE_DIR} and
# to the root binary directory of the project as ${MRROCPP_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)
project (MRROCPP)

set(CMAKE_MODULE_PATH ${MRROCPP_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})


include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build.")

# include module for dependent option
include(CMakeDependentOption)




# set short names for project directories
set (ECP_MPDIR ${PROJECT_SOURCE_DIR}/src/ecp_mp)
set (ECPDIR ${PROJECT_SOURCE_DIR}/src/ecp)

# set default install prefix
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
      ${PROJECT_SOURCE_DIR} CACHE PATH "${CMAKE_PROJECT_NAME} install prefix" FORCE
  )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE
  )
ENDIF(NOT CMAKE_BUILD_TYPE)

# Recurse into the "src" subdirectories.
add_subdirectory (src)
