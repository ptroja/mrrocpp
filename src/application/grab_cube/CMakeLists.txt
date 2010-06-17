if(ROBOT_IRP6_ON_TRACK)
add_executable(ecp_t_grab_cube_irp6ot
	ecp_g_vis_ib_eih_wrist_turner_irp6ot.cc
	ecp_t_grab_cube_irp6ot.cc
	${ECPDIR}/irp6ot_m/generator/ecp_g_vis_ib_eih_object_tracker_irp6ot.cc
)

target_link_libraries(ecp_t_grab_cube_irp6ot
	ecp_r_irp6ot_m
	ecp
)

install(TARGETS ecp_t_grab_cube_irp6ot DESTINATION bin)
endif(ROBOT_IRP6_ON_TRACK)
