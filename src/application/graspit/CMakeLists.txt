add_executable(ecp_irp6_graspit
	ecp_t_irp6_graspit.cc
)

target_link_libraries(ecp_irp6_graspit
	ecp_r_irp6ot_m
	ecp_r_irp6p_m
	ecp
)

add_executable(ecp_birdhand_graspit
	ecp_t_birdhand_graspit.cc
	ecp_g_birdhand_graspit.cc
)

target_link_libraries(ecp_birdhand_graspit
	ecp_r_bird_hand
	ecp
)

add_executable(mp_tfg_graspit
	mp_t_tfg_graspit.cc
	ecp_mp_tr_graspit.cc
)

target_link_libraries(mp_tfg_graspit
	mp
)

add_executable(mp_birdhand_graspit
	mp_t_birdhand_graspit.cc
	ecp_mp_tr_graspit.cc
)

target_link_libraries(mp_birdhand_graspit
	mp
)

install(TARGETS mp_tfg_graspit mp_birdhand_graspit ecp_irp6_graspit ecp_birdhand_graspit DESTINATION bin)
