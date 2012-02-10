add_subdirectory (weight_measure)
add_subdirectory (bias_edp_force)
add_subdirectory (force_tool_change)
add_subdirectory (tff_gripper_approach)
add_subdirectory (tff_nose_run)
add_subdirectory (tff_rubik_face_rotate)
add_subdirectory (transparent)
add_subdirectory (sleep)
add_subdirectory (smooth_file_from_mp)
add_subdirectory (newsmooth)
add_subdirectory (get_position)
add_subdirectory (constant_velocity)
add_subdirectory (spline)
add_subdirectory (visual_servo)
add_subdirectory (teach_in)
add_subdirectory (operator_reaction_condition)


add_library(ecp_mp_generators
ecp_mp_g_common.cc
)


target_link_libraries(ecp_mp_generators 
ecp_mp_g_tff_nose_run
ecp_mp_g_tff_gripper_approach
)


add_library(ecp_generators
	ecp_g_common.cc
)

target_link_libraries(ecp_generators 
ecp_g_operator_reaction_condition
ecp_g_teach_in
ecp_g_weight_measure
ecp_g_bias_edp_force 
ecp_g_force_tool_change
ecp_g_tff_gripper_approach
ecp_g_tff_nose_run
ecp_g_tff_rubik_face_rotate
ecp_g_transparent
ecp_g_sleep
ecp_g_smooth_file_from_mp
ecp_g_newsmooth
ecp_smooth_base
ecp_g_constant_velocity
ecp_g_get_position
ecp_g_spline
ecp_g_visual_servo
)


target_link_libraries(ecp_generators ecp)

install(TARGETS ecp_generators ecp_mp_generators DESTINATION lib)

