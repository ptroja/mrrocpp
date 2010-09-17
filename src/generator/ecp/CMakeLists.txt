add_library(ecp_generators
	../../generator/ecp/ecp_g_teach_in.cc
	../../generator/ecp/ecp_g_delta.cc
	../../generator/ecp/ecp_g_operator_reaction_condition.cc
	../../generator/ecp/ecp_g_jarosz.cc
	../../generator/ecp/ecp_g_visual_servo_manager.cc
	../../generator/ecp/ecp_g_visual_servo.cc
	../../generator/ecp/ecp_g_smooth.cc
	../../generator/ecp/ecp_g_sleep.cc
	../../generator/ecp/ecp_g_force.cc
	../../generator/ecp/ecp_g_get_position.cc
	../../generator/ecp/ecp_g_constant_velocity.cc
	../../generator/ecp/ecp_g_newsmooth.cc
	../../generator/ecp/velocity_profile_calculator/bang_bang_profile.cc
	../../generator/ecp/velocity_profile_calculator/constant_velocity_profile.cc
	../../generator/ecp/trajectory_interpolator/constant_velocity_interpolator.cc
	../../generator/ecp/trajectory_interpolator/bang_bang_interpolator.cc
)

# @bug cyclic dependency between ecp<->ecp_generators prevents building as
# shared libraries // ptroja
target_link_libraries(ecp_generators mrrocpp ecp)
