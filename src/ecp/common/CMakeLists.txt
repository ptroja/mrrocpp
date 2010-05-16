add_library(ecp
	ECP_main_error.cc
	ecp_robot.cc
	ecp_m.cc
	task/ecp_task.cc
	generator/ecp_generator.cc
	generator/ecp_g_teach_in.cc
	generator/ecp_g_delta.cc
	generator/ecp_g_operator_reaction_condition.cc
	generator/ecp_g_jarosz.cc
	generator/ecp_g_visual_servo_manager.cc
	generator/ecp_g_visual_servo.cc
	ecp_taught_in_pose.cc
	generator/ecp_g_smooth.cc
	generator/ecp_g_sleep.cc
	generator/ecp_g_force.cc
	generator/ecp_g_transparent.cc
	generator/ecp_g_get_position.cc
	generator/velocity_profile_calculator/bang_bang_profile.cc
)

target_link_libraries(ecp ecp_mp)
