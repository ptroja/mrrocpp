add_executable(ecp_irp6_graspit
	ecp_t_irp6_graspit.cc
	ecp_mp_tr_graspit.cc
)

target_link_libraries(ecp_irp6_graspit
	ecp_r_irp6ot_m ecp_r_irp6p_m
	ecp
)

add_executable(mp_graspit
	mp_t_graspit.cc
	ecp_mp_tr_graspit.cc
)

target_link_libraries(mp_graspit
	mp
)

install(TARGETS mp_graspit ecp_irp6_graspit DESTINATION bin)
