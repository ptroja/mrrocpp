add_executable(ecp_graspit
	ecp_mp_tr_graspit.cc
	ecp_t_graspit.cc
)

target_link_libraries(ecp_graspit
	ecp_r_irp6ot_m ecp_r_irp6p_m
	ecp
)

install(TARGETS ecp_graspit DESTINATION bin)
