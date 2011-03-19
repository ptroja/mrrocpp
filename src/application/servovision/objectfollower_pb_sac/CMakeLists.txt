# object follower - PB - SAC
add_executable(ecp_objectfollower_pb_sac
    ecp_t_objectfollower_pb_sac.cc
    ../pb_sac_visual_servo.cc
    ../pb_visual_servo.cc
    ../visual_servo.cc
    ../visual_servo_manager.cc
    ../single_visual_servo_manager.cc
    ../position_constraint.cc
    ../cubic_constraint.cc
    ../termination_condition.cc
)

target_link_libraries(ecp_objectfollower_pb_sac
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

####################################3
install(TARGETS ecp_objectfollower_pb_sac DESTINATION bin)
