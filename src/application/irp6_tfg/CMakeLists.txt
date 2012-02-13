add_executable(ecp_tfg
	ecp_g_tfg.cc
	ecp_g_constant_velocity_tfg.cc
	ecp_t_tfg.cc
)

target_link_libraries(ecp_tfg
	ecp_robot
)

install(TARGETS ecp_tfg DESTINATION bin)

