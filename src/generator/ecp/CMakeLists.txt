add_subdirectory (force)

add_library(ecp_generators
	ecp_g_teach_in.cc
	ecp_g_delta.cc
	ecp_g_operator_reaction_condition.cc
        ecp_g_jarosz.cc
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

target_link_libraries(ecp_generators ecp_generators_force mrrocpp ecp)

install(TARGETS ecp_generators DESTINATION lib)
