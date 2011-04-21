# TODO: remove these utility macros to separate file
MACRO(TARGET_LINK_LIBRARY_IF CONDITION TARGET LIBRARY)
	set(CONDITION_VALUE ${${CONDITION}})
	if(CONDITION_VALUE)
		target_link_libraries(${TARGET} ${LIBRARY})
	endif(CONDITION_VALUE)
ENDMACRO(TARGET_LINK_LIBRARY_IF)

OPTION (BUILD_SHARED_LIBS "Build shared libraries." OFF)

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

option(ROBOT_CONVEYOR "Build support for conveyor robot" ON)
option(ROBOT_IRP6_ON_TRACK "Build support for IRp6 'on track' robot" ON)
CMAKE_DEPENDENT_OPTION(ROBOT_IRP6_ON_TRACK_M
	"Build support for IRp6 'on track' robot without a gripper"
	ON "ROBOT_IRP6_ON_TRACK" OFF)
CMAKE_DEPENDENT_OPTION(ROBOT_IRP6_ON_TRACK_TFG
	"Build support for gripper mounted on IRp6 'on track' robot"
	ON "ROBOT_IRP6_ON_TRACK" OFF)
option(ROBOT_IRP6_POSTUMENT "Build support for IRp6 'postument' robot" ON)
CMAKE_DEPENDENT_OPTION(ROBOT_IRP6_POSTUMENT_M
	"Build support for IRp6 'postument' robot without a gripper"
	ON "ROBOT_IRP6_POSTUMENT" OFF)
CMAKE_DEPENDENT_OPTION(ROBOT_IRP6_POSTUMENT_TFG
	"Build support for gripper mounted on IRp6 'postument' robot"
	ON "ROBOT_IRP6_POSTUMENT" OFF)
option(ROBOT_IRP6_MECHATRONICS "Build support for IRp6 robot located at Mechatronics Deptartment" ON)
option(ROBOT_POLYCRANK "Build support for Polycrank robot" ON)


option(ROBOTS_SWARMITFIX "Build support for SwarmItFIX robots" ON)
CMAKE_DEPENDENT_OPTION(ROBOT_SHEAD
	"Build support for SwarmItFIX 'head' robot"
	ON "ROBOTS_SWARMITFIX" OFF)
CMAKE_DEPENDENT_OPTION(ROBOT_SMB
	"Build support for SwarmItFIX mobile base robot"
	ON "ROBOTS_SWARMITFIX" OFF)
CMAKE_DEPENDENT_OPTION(ROBOT_SPKM
	"Build support for SwarmItFIX parallel kinematics machine robot"
	ON "ROBOTS_SWARMITFIX" OFF)
	
option(ROBOT_BIRD_HAND "Build support for three finger robot hand" ON)
	
option(ROBOT_SARKOFAG "Build support for sarkofag" ON)



if (UBUNTU32BIT)
	# QNX headers and libraries are kept in 'mrlib' repository
	set(UBUNTU32BIT_ROOT "/opt/ubuntu-10.10-32bit")
	set(MRLIB "${UBUNTU32BIT_ROOT}/usr")

	set(EIGEN2_INCLUDE_DIRS "${MRLIB}/include/eigen2") 
	set(LIBXML2_INCLUDE_DIR "${MRLIB}/include/libxml2")
	#set(Boost_INCLUDE_DIR "${MRLIB}/include")

	# Boost is provided by mrlib repository
	set(BOOST_ROOT "${MRLIB}")
	find_package(Boost 1.41 COMPONENTS thread serialization filesystem system date_time)

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

	# Warning messages (GCC)
	set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare -Werror=return-type")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare -Werror=return-type")

	# keep this flag in production code
	add_definitions (-DEIGEN_DONT_ALIGN)

	# cross-compiling linker flags
	set(CMAKE_EXE_LINKER_FLAGS "-Wl,--rpath-link=${UBUNTU32BIT_ROOT}/lib:${MRLIB}/lib")
	set(CMAKE_SKIP_RPATH ON)

	# Tips for finding cross-libraries and headers
	include_directories (${MRLIB}/include)
	set(COMEDI_INSTALL "${MRLIB}")
else (UBUNTU32BIT) # Linux, MacOS, etc.
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

	# Warning messages (GCC)
	set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare -Werror=return-type")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare -Werror=return-type")

	# keep this flag until we fix alignment 'new' operator issue
	add_definitions (-DEIGEN_DONT_ALIGN)
endif (UBUNTU32BIT)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/src/config.h.in"
  "${PROJECT_BINARY_DIR}/src/config.h"
)
# add include path to the generated config file
include_directories (${MRROCPP_BINARY_DIR}/src) 

# Boost is a common requirements for all the platforms
find_package(Boost 1.41 COMPONENTS thread serialization filesystem system date_time)



include(FindComedi)

# Make sure the compiler can find include files
include_directories (${MRROCPP_SOURCE_DIR}/src) 
include_directories (${LIBXML2_INCLUDE_DIR})
include_directories (${Boost_INCLUDE_DIR})
include_directories (${EIGEN2_INCLUDE_DIRS})


	include_directories (${PROJECT_SOURCE_DIR}/src/lib/messip)
	set(COMMON_LIBRARIES messip)


set(COMMON_LIBRARIES mrrocpp ${COMMON_LIBRARIES}
	${Boost_THREAD_LIBRARY}
	${COMPATIBILITY_LIBRARIES}
)

# Recurse into subdirectories.
add_subdirectory (base)
add_subdirectory (sensor)
add_subdirectory (generator)
add_subdirectory (subtask)
add_subdirectory (robot)
add_subdirectory (application)
add_subdirectory (ui-qt)
