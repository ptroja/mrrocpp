

# TODO: remove these utility macros to separate file
MACRO(TARGET_LINK_LIBRARY_IF CONDITION TARGET LIBRARY)
	set(CONDITION_VALUE ${${CONDITION}})
	if(CONDITION_VALUE)
		target_link_libraries(${TARGET} ${LIBRARY})
	endif(CONDITION_VALUE)
ENDMACRO(TARGET_LINK_LIBRARY_IF)

# the RPATH to be used when installing
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

## this is currently disabled
## as CMake option names duplicates _ROBOT_ENUM values
# define both CMake option and -D flag for compiler
#MACRO(OPTION_AND_DEFINITION OPTION DESCRIPTION DEFAULT_VALUE)
#	option(${OPTION} ${DESCRIPTION} ${DEFAULT_VALUE})
#	set(OPTION_VALUE ${${OPTION}})
#	if(OPTION_VALUE)
#		add_definitions("-D${OPTION}")
#	endif(OPTION_VALUE)
#ENDMACRO(OPTION_AND_DEFINITION)




option(ROBOTS_SWARMITFIX "Build support for SwarmItFIX robots" ON)

option(ROBOTS_012 "Build support for 012 room robots" ON)
	
option(ROBOT_BIRD_HAND "Build support for three finger robot hand" ON)
	
option(UI_QT "Build support for UI written in QT" ON)	

if (UBUNTU32BIT)
	# 32-bit headers and libraries are kept in 'mrlib' repository
	set(UBUNTU32BIT_ROOT "/opt/ubuntu-10.04-32bit")
	set(MRLIB "${UBUNTU32BIT_ROOT}/usr")

	set(EIGEN2_INCLUDE_DIRS "${MRLIB}/include/eigen2") 
	set(LIBXML2_INCLUDE_DIR "${MRLIB}/include/libxml2")
	#set(Boost_INCLUDE_DIR "${MRLIB}/include")

	# Boost is provided by mrlib repository
	set(BOOST_ROOT "${MRLIB}")

	# FIXME: define libxml2 libraries to use
	find_library(LIBXML2_LIBRARY xml2 "${MRLIB}/lib")
	set(LIBXML2_LIBRARIES ${LIBXML2_LIBRARY})

	# Check for libftdi without pkg-config
	find_library(LibFTDI_LIBRARY ftdi "${MRLIB}/lib")
	set(LibFTDI_LIBRARIES ${LibFTDI_LIBRARY})
	set(LibFTDI_LDFLAGS "${MRLIB}/lib/libftdi.so")
	set(LibFTDI_FOUND 1)

	# clock_gettime() can be missing in -lc
	INCLUDE(CheckLibraryExists)
	CHECK_LIBRARY_EXISTS (rt clock_gettime "" HAVE_LIBRT)
	if(HAVE_LIBRT)
		set (COMPATIBILITY_LIBRARIES rt)
	endif(HAVE_LIBRT)

	# (cross) compiling for 32 bit target
	set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -m32")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")

	# keep this flag in production code
	add_definitions (-DEIGEN_DONT_ALIGN)

	# cross-compiling linker flags
	set(CMAKE_EXE_LINKER_FLAGS "-Wl,--rpath-link=${UBUNTU32BIT_ROOT}/lib:${MRLIB}/lib")
	set(CMAKE_SKIP_RPATH ON)

	# Tips for finding cross-libraries and headers
	include_directories (${MRLIB}/include)
	set(COMEDI_INSTALL "${MRLIB}")
else (UBUNTU32BIT) # Linux, MacOS, etc.
	# Build shared libraries by default
	OPTION (BUILD_SHARED_LIBS "Build shared libraries." ON)

	# find dependency packages
	set(BOOST_ROOT "/opt/boost")

	find_package(Threads REQUIRED)
	find_package(LibXml2 REQUIRED)
	
	# Check for libftdi with pkg-config
	include(FindPkgConfig)
	pkg_check_modules(LibFTDI libftdi)
	
	# Eigen2 is not directly supported by CMake
	pkg_check_modules(EIGEN2 REQUIRED eigen2)

	# clock_gettime() can be missing in -lc
	CHECK_LIBRARY_EXISTS (rt clock_gettime "" HAVE_LIBRT)
	if(HAVE_LIBRT)
		set (COMPATIBILITY_LIBRARIES rt)
	endif(HAVE_LIBRT)

	# check for kqueue()
	INCLUDE (CheckSymbolExists)
	CHECK_SYMBOL_EXISTS(kqueue sys/event.h HAVE_KQUEUE)
	CHECK_SYMBOL_EXISTS(mlockall sys/mman.h HAVE_MLOCKALL)

	# keep this flag until we fix alignment 'new' operator issue
	add_definitions (-DEIGEN_DONT_ALIGN)
endif (UBUNTU32BIT)

# Warning messages (GCC)
if(CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -Werror=return-type")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -Werror=return-type")
endif(CMAKE_COMPILER_IS_GNUCXX)	

# Boost is a common requirements for all the platforms
find_package(Boost 1.41 COMPONENTS thread serialization filesystem system date_time)

# Enable C++0x extensions
if(CMAKE_COMPILER_IS_GNUCXX)
    # TODO: check against differrent Boost versions
    if("${Boost_VERSION}" LESS "104601")
        message(STATUS "Applying C++0x Boost configuration fix")
	   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_COMPILER_CONFIG=\"\\\"gcc-boost-c++0x.hpp\\\"\"")
    else("${Boost_VERSION}" LESS "104601")
        message(STATUS "C++0x Boost configuration fix not required")
    endif("${Boost_VERSION}" LESS "104601")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX)

# Check for external libraries.
include(FindComedi)
include(FindXSD)
include(FindXerces)
pkg_check_modules(ZeroMQ libzmq)

# Make sure the compiler can find include files
include_directories (${MRROCPP_SOURCE_DIR}/src) 
include_directories (SYSTEM ${LIBXML2_INCLUDE_DIR})
include_directories (SYSTEM ${Boost_INCLUDE_DIR})
include_directories (SYSTEM ${EIGEN2_INCLUDE_DIRS})

include_directories (${PROJECT_SOURCE_DIR}/src/lib/messip)

set(COMMON_LIBRARIES mrrocpp messip
	${Boost_THREAD_LIBRARY}
	${COMPATIBILITY_LIBRARIES}
)

if(ROBOT_BIRD_HAND)
set (R_BIRD_HAND 1)
else(ROBOT_BIRD_HAND)
set (R_BIRD_HAND 0)
endif(ROBOT_BIRD_HAND)

if(ROBOTS_SWARMITFIX)
set (R_SWARMITFIX 1)
else(ROBOTS_SWARMITFIX)
set (R_SWARMITFIX 0)
endif(ROBOTS_SWARMITFIX)

if(ROBOTS_012)
set (R_012 1)
else(ROBOTS_012)
set (R_012 0)
endif(ROBOTS_012)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/src/config.h.in"
  "${PROJECT_BINARY_DIR}/src/config.h"
)

# add include path to the generated config file
include_directories (${MRROCPP_BINARY_DIR}/src) 

# Recurse into subdirectories.
add_subdirectory (base)
add_subdirectory (sensor)
add_subdirectory (generator)
add_subdirectory (subtask)
add_subdirectory (robot)
add_subdirectory (application)
add_subdirectory (contrib)
if(UI_QT)
add_subdirectory (ui-qt)
endif(UI_QT)
