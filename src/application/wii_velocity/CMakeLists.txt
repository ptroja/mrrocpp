add_executable(ecp_wii_velocity
	ecp_t_wii_velocity.cc
	generator/ecp_g_wii_velocity.cc
	../wii_teach/sensor/ecp_mp_s_wiimote.cc
)

target_link_libraries(ecp_wii_velocity
	ecp_r_irp6ot_m
	ecp
)

install(TARGETS ecp_wii_velocity DESTINATION bin)
