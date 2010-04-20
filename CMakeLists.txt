# The name of our project is "MRROCPP". CMakeLists files in this project can
# refer to the root source directory of the project as ${MRROCPP_SOURCE_DIR} and
# to the root binary directory of the project as ${MRROCPP_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)
project (MRROCPP)

find_package(Boost 1.39 COMPONENTS thread )
find_package(Threads REQUIRED)
find_package(LibXml2 REQUIRED)

# Make sure the compiler can find include files
include_directories (${MRROCPP_SOURCE_DIR}/src) 
include_directories (${LIBXML2_INCLUDE_DIR})

#set (ECP_MPDIR ${PROJECT_SOURCE_DIR}/src/ecp_mp)
# QNX is default MRROC++ target
if (QNXNTO)
	add_definitions (-DEIGEN_DONT_ALIGN)
else (QNXNTO) # Linux, etc.
	include_directories (/usr/include/eigen2) 
	add_definitions (-DUSE_MESSIP_SRR)
	CHECK_LIBRARY_EXISTS (rt clock_gettime "" HAVE_LIBRT)
	if(HAVE_LIBRT)
		set (COMPATIBILITY_LIBS ${COMPATIBILITY_LIBS} rt)
	endif(HAVE_LIBRT)
	set(COMMON_LIBS messip)
endif (QNXNTO)

set(COMMON_LIBS mrrocpp ${COMMON_LIBS} ${LIBXML2_LIBRARIES} ${Boost_LIBRARIES})

# Recurse into the "src" subdirectories.
add_subdirectory (src)
