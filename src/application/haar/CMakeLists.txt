if(ROBOT_IRP6_ON_TRACK)
add_executable(ecp_haar
	ecp_t_haar_irp6ot.cc
	ecp_g_rotate_gripper.cc
)

target_link_libraries(ecp_haar
	ecp_r_irp6ot_m
	ecp
)

install(TARGETS ecp_haar DESTINATION bin)
endif(ROBOT_IRP6_ON_TRACK)
