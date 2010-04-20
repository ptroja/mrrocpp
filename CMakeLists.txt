# The name of our project is "MRROCPP". CMakeLists files in this project can
# refer to the root source directory of the project as ${MRROCPP_SOURCE_DIR} and
# to the root binary directory of the project as ${MRROCPP_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)
project (MRROCPP)

find_package( Boost 1.39 COMPONENTS thread )
find_package(Threads REQUIRED)

# Make sure the compiler can find include files from our Hello library.
include_directories (${MRROCPP_SOURCE_DIR}/src) 

# QNX is default MRROC++ target
if (QNXNTO)
	set(COMMON_LIBS ${Boost_LIBRARIES})
else (QNXNTO) # Linux, etc.
	include_directories (/usr/include/eigen2) 
	add_definitions (-DUSE_MESSIP_SRR)
	CHECK_LIBRARY_EXISTS (rt clock_gettime "" HAVE_LIBRT)
	if(HAVE_LIBRT)
		set (COMPATIBILITY_LIBS ${COMPATIBILITY_LIBS} rt)
	endif(HAVE_LIBRT)
	set(COMMON_LIBS messip ${Boost_LIBRARIES})
endif (QNXNTO)

# Recurse into the "src" subdirectories.
add_subdirectory (src)
