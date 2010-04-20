add_executable(ecp_t_grab_cube_irp6ot
	ecp_g_vis_ib_eih_wrist_turner_irp6ot.cc
	ecp_t_grab_cube_irp6ot.cc
	${ECPDIR}/irp6_on_track/generator/ecp_g_vis_ib_eih_object_tracker_irp6ot.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_cvfradia.cc
)

target_link_libraries(ecp_t_grab_cube_irp6ot
	ecp_r_irp6ot ecp_r_irp6p
	kinematicsirp6ot kinematicsirp6p kinematics
	ecp ecp_mp ${COMMON_LIBS}
)

install(TARGETS ecp_t_grab_cube_irp6ot RUNTIME DESTINATION bin)
