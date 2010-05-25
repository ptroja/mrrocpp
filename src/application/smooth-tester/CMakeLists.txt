if(ROBOT_IRP6_ON_TRACK AND ROBOT_IRP6_POSTUMENT)
add_executable(ecp_smooth_test
	ecp_t_smooth_test.cc
)

target_link_libraries(ecp_smooth_test
	ecp_r_irp6p_m ecp_r_irp6ot_m
	ecp
)

install(TARGETS ecp_smooth_test DESTINATION bin)
endif(ROBOT_IRP6_ON_TRACK AND ROBOT_IRP6_POSTUMENT)
