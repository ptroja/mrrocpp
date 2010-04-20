# The name of our project is "MRROCPP". CMakeLists files in this project can
# refer to the root source directory of the project as ${MRROCPP_SOURCE_DIR} and
# to the root binary directory of the project as ${MRROCPP_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)
project (MRROCPP)

find_package( Boost 1.39 COMPONENTS thread )
find_package(Threads REQUIRED)

CHECK_LIBRARY_EXISTS (rt clock_gettime "" HAVE_LIBRT)
if(HAVE_LIBRT)
	set (COMPATIBILITY_LIBS ${COMPATIBILITY_LIBS} rt)
endif(HAVE_LIBRT)

# Recurse into the "src" subdirectories.
add_subdirectory (src)
