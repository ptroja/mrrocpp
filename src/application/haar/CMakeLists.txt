if(ROBOT_IRP6_ON_TRACK)
add_executable(ecp_haar
	ecp_t_haar_irp6ot.cc
	ecp_g_rotate_gripper.cc
	${ECPDIR}/irp6ot_m/generator/ecp_g_vis_ib_eih_planar_irp6ot.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_cvfradia.cc
)

target_link_libraries(ecp_haar
	ecp_r_irp6ot_m
	ecp
)

install(TARGETS ecp_haar DESTINATION bin)
endif(ROBOT_IRP6_ON_TRACK)
