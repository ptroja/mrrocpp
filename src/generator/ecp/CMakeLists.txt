
add_subdirectory (force)
add_subdirectory (bias_edp_force)
add_subdirectory (force_tool_change)
add_subdirectory (tff_gripper_approach)

add_library(ecp_mp_generators
ecp_mp_g_common.cc
)


target_link_libraries(ecp_mp_generators 
ecp_mp_generators_force
ecp_mp_g_tff_gripper_approach
)

add_library(ecp_generators
	../../base/ecp/ecp_taught_in_pose.cc
	../../base/ecp/ecp_g_transparent.cc
	ecp_g_teach_in.cc
	ecp_g_delta.cc
	ecp_g_operator_reaction_condition.cc
	ecp_g_visual_servo_manager.cc
	ecp_g_visual_servo.cc
	ecp_g_sleep.cc
	ecp_g_get_position.cc
	ecp_g_constant_velocity.cc
	ecp_g_newsmooth.cc
        ecp_g_spline.cc
	velocity_profile_calculator/bang_bang_profile.cc
	velocity_profile_calculator/constant_velocity_profile.cc
        velocity_profile_calculator/spline_profile.cc
	trajectory_interpolator/constant_velocity_interpolator.cc
	trajectory_interpolator/bang_bang_interpolator.cc
        trajectory_interpolator/spline_interpolator.cc
)

target_link_libraries(ecp_generators 
ecp_generators_force 
ecp_g_bias_edp_force 
ecp_g_force_tool_change
ecp_g_tff_gripper_approach
)



install(TARGETS ecp_generators ecp_mp_generators DESTINATION lib)
