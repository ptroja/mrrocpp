add_library(ecp_g_visual_servo
	ecp_g_visual_servo.cc
		ecp_g_visual_servo_manager.cc
)

target_link_libraries(ecp_g_visual_servo ecp_mp_generators ecp_mp)

install(TARGETS ecp_g_visual_servo DESTINATION lib)
