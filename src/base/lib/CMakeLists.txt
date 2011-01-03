if(USE_MESSIP)
	add_subdirectory (messip)

	# configsrv is needed only for non-QNX platforms and then messip is used
	add_executable(configsrv configsrv.cc configsrv_m.cc)
	target_link_libraries (configsrv messip ${Boost_THREAD_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
	install(TARGETS configsrv DESTINATION bin)
endif(USE_MESSIP)

add_subdirectory (agent)
add_subdirectory (sr)

# Create a library called "mrrocpp" which includes the following source files.
# The extension is already found. Any number of sources could be listed here. 

add_library(mrrocpp 
	mrmath/ft_v_tr.cc
	mrmath/ForceTrans.cc
	mrmath/homog_matrix.cc
	mrmath/mathtr.cc
	mrmath/k_vector.cc
	mrmath/ft_v_vector.cc
	mrmath/jacobian_matrix.cc
	configurator.cc
	c_buffer.cc r_buffer.cc
	mis_fun.cc
	timer.cc
	exception.cc
	playerpos_goal.cc
	datastr.cc
	condition_synchroniser.cc
	single_thread_port.cc
	logger.cc
	trajectory_pose/bang_bang_trajectory_pose.cc
	trajectory_pose/trajectory_pose.cc
	trajectory_pose/constant_velocity_trajectory_pose.cc
)

target_link_libraries (mrrocpp sr)

if(NOT USE_MESSIP)
	target_link_libraries (mrrocpp ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
endif(NOT USE_MESSIP)

if(USE_MESSIP)
    target_link_libraries(mrrocpp ${Boost_SERIALIZATION_LIBRARY})
endif(USE_MESSIP)
