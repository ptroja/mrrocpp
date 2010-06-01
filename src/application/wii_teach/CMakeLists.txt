add_executable(ecp_wii_teach
	ecp_t_wii_teach.cc
	generator/ecp_g_wii_absolute.cc
	generator/ecp_g_wii_joint.cc
	generator/ecp_g_wii_relative.cc
	generator/ecp_g_wii.cc
	sensor/ecp_mp_s_wiimote.cc
)

target_link_libraries(ecp_wii_teach
	ecp_r_irp6ot_m
	ecp
)

install(TARGETS ecp_wii_teach DESTINATION bin)
