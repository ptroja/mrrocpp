add_subdirectory (messip)
add_subdirectory (agent)

# Make sure the compiler can find include files from our Hello library.
include_directories (${MRROCPP_SOURCE_DIR}/src) 

# Linux
include_directories (/usr/include/eigen2) 
add_definitions (-DUSE_MESSIP_SRR)
# QNX: TODO

# Create a library called "mrrocpp" which includes the source file "hello.cxx".
# The extension is already found. Any number of sources could be listed here. 

add_library(mrrocpp 
	mrmath/ft_v_tr.cc
	mrmath/ForceTrans.cc
	mrmath/homog_matrix.cc
	mrmath/mathtr.cc
	mrmath/k_vector.cc
	mrmath/ft_v_vector.cc
	mrmath/jacobian_matrix.cc
	cfgopts.cc
	configurator.cc
	c_buffer.cc r_buffer.cc
	mis_fun.cc
	timer.cc
	srlib.cc
	exception.cc
	playerpos_goal.cc
	datastr.cc
)

add_executable(configsrv configsrv.cc cfgopts.cc configsrv_m.cc)
target_link_libraries (configsrv messip ${Boost_LIBRARIES})
	
	
