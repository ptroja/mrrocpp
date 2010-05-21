if(USE_MESSIP)
	add_subdirectory (messip)

	# configsrv is needed only for non-QNX platforms and then messip is used
	add_executable(configsrv configsrv.cc cfgopts.cc configsrv_m.cc)
	target_link_libraries (configsrv messip ${Boost_THREAD_LIBRARY})
	install(TARGETS configsrv DESTINATION bin)
endif(USE_MESSIP)

add_subdirectory (agent)

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
	epos_gen.cc
	timer.cc
	srlib.cc
	exception.cc
	playerpos_goal.cc
	datastr.cc
	condition_synchroniser.cc
	logger.cc
)
