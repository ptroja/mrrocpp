# TODO: remove these utility macros to separate file
MACRO(TARGET_LINK_LIBRARY_IF CONDITION TARGET LIBRARY)
	set(CONDITION_VALUE ${${CONDITION}})
	if(CONDITION_VALUE)
		target_link_libraries(${TARGET} ${LIBRARY})
	endif(CONDITION_VALUE)
ENDMACRO(TARGET_LINK_LIBRARY_IF)

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
option(ROBOT_SPEAKER "Build support for speaker robot" ON)

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
	
CMAKE_DEPENDENT_OPTION(ROBOT_BIRD_HAND
	"Build support for three finger robot hand"
	ON "ROBOTS_SWARMITFIX" OFF)

option(USE_MESSIP "Use MESSIP TCP messaging protocol" OFF)

# QNX is default MRROC++ target
if (QNXNTO)
	# QNX headers and libraries are kept in 'mrlib' repository
	set(MRLIB "$ENV{QNX_TARGET}/mrlib")

	# keep this flag until bug in QNX stack alignment is fixed
	add_definitions (-DEIGEN_DONT_ALIGN)	
	set(EIGEN2_INCLUDE_DIR "${MRLIB}/include/eigen2") 
	set(LIBXML2_INCLUDE_DIR "${MRLIB}/include")
	set(Boost_INCLUDE_DIR "${MRLIB}/include")

	# Boost is provided by mrlib repository
	set(BOOST_ROOT "${MRLIB}")
	find_package(Boost 1.39 COMPONENTS thread serialization)
	# include boost compatibility layer
	include_directories ("${Boost_INCLUDE_DIRS}/boost/compatibility/cpp_c_headers") 

	# FIXME: define libxml2 libraries to use
	find_library(LIBXML2_LIBRARY xml2 "${MRLIB}/lib")
	find_library(LIBICONV_LIBRARY iconv "${MRLIB}/lib")
	set(LIBXML2_LIBRARIES ${LIBXML2_LIBRARY} ${LIBICONV_LIBRARY})
	
	# QNX provides networking with separate library
	set (COMPATIBILITY_LIBRARIES socket m)
else (QNXNTO) # Linux, etc.
	# find dependency packages
	find_package(Boost 1.39 COMPONENTS thread serialization)
	find_package(Threads REQUIRED)
	find_package(LibXml2 REQUIRED)

	# Eigen2 is not supported by CMake
	set(EIGEN2_INCLUDE_DIR /usr/include/eigen2) 

	# clock_gettime() can be missing in -lc
	CHECK_LIBRARY_EXISTS (rt clock_gettime "" HAVE_LIBRT)
	if(HAVE_LIBRT)
		set (COMPATIBILITY_LIBRARIES rt)
	endif(HAVE_LIBRT)

	# for non-QNX target messip is the only messaging protocol
	set(USE_MESSIP ON)
endif (QNXNTO)

# Make sure the compiler can find include files
include_directories (${MRROCPP_SOURCE_DIR}/src) 
include_directories (${LIBXML2_INCLUDE_DIR})
include_directories (${Boost_INCLUDE_DIR})
include_directories (${EIGEN2_INCLUDE_DIR}) 

if(USE_MESSIP)
	include_directories (${PROJECT_SOURCE_DIR}/src/lib/messip)
	add_definitions (-DUSE_MESSIP_SRR)
	set(COMMON_LIBRARIES messip)
endif(USE_MESSIP)

set(COMMON_LIBRARIES mrrocpp ${COMMON_LIBRARIES}
	${Boost_THREAD_LIBRARY}
	${COMPATIBILITY_LIBRARIES}
)

# Recurse into subdirectories.
add_subdirectory (lib)
add_subdirectory (kinematics)
add_subdirectory (edp)
add_subdirectory (ecp)
add_subdirectory (ecp_mp)
add_subdirectory (mp)
add_subdirectory (ui)
add_subdirectory (application)
add_subdirectory (vsp)
