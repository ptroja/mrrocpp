# The name of our project is "MRROCPP". CMakeLists files in this project can
# refer to the root source directory of the project as ${MRROCPP_SOURCE_DIR} and
# to the root binary directory of the project as ${MRROCPP_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)
project (MRROCPP)

set (ECP_MPDIR ${PROJECT_SOURCE_DIR}/src/ecp_mp)
set (ECPDIR ${PROJECT_SOURCE_DIR}/src/ecp)

# QNX is default MRROC++ target
if (QNXNTO)
	# keep this flag until bug in QNX is fixed
	add_definitions (-DEIGEN_DONT_ALIGN)

	# QNX headers and libraries are kept in 'mrlib' repository
	include_directories ($ENV{QNX_TARGET}/mrlib/include/eigen2) 
	set(LIBXML2_INCLUDE_DIR $ENV{QNX_TARGET}/mrlib/include)
	set(Boost_INCLUDE_DIR $ENV{QNX_TARGET}/mrlib/include)

	# Eigen2 is not supported by CMake
	include_directories ($ENV{QNX_TARGET}/mrlib/include/eigen2) 

	# FIXME: define boost libraries to use
	find_library(Boost_LIBRARIES boost_thread-mt $ENV{QNX_TARGET}/mrlib/lib)
	# include boost compatibility layer
	include_directories ($ENV{QNX_TARGET}/mrlib/include/boost/compatibility/cpp_c_headers) 

	# FIXME: define libxml2 libraries to use
	find_library(LIBXML2_LIBRARIES xml2 $ENV{QNX_TARGET}/mrlib/lib)
	find_library(LIBICONV_LIBRARIES iconv $ENV{QNX_TARGET}/mrlib/lib)
	set(LIBXML2_LIBRARIES ${LIBXML2_LIBRARIES} ${LIBICONV_LIBRARIES})

	# QNX provides networking with separate library
	set (COMPATIBILITY_LIBRARIES socket m)
else (QNXNTO) # Linux, etc.
	# find dependency packages
	find_package(Boost 1.39 COMPONENTS thread )
	find_package(Threads REQUIRED)
	find_package(LibXml2 REQUIRED)

	# Eigen2 is not supported by CMake
	include_directories (/usr/include/eigen2) 

	# clock_gettime() can be missing in -lc
	CHECK_LIBRARY_EXISTS (rt clock_gettime "" HAVE_LIBRT)
	if(HAVE_LIBRT)
		set (COMPATIBILITY_LIBRARIES rt)
	endif(HAVE_LIBRT)

	# for non-QNX target messip is the only messaging protocol
	add_definitions (-DUSE_MESSIP_SRR)
	set(COMMON_LIBS messip)
endif (QNXNTO)

# Make sure the compiler can find include files
include_directories (${MRROCPP_SOURCE_DIR}/src) 
include_directories (${LIBXML2_INCLUDE_DIR})
include_directories (${Boost_INCLUDE_DIR})

set(COMMON_LIBS mrrocpp ${COMMON_LIBS}
	${COMPATIBILITY_LIBRARIES}
	#${LIBXML2_LIBRARIES}
	${Boost_LIBRARIES}
)

# Recurse into the "src" subdirectories.
add_subdirectory (src)
