# object follower - PB - EIH
add_executable(ecp_objectfollower_pb_eih
    ecp_t_objectfollower_pb_eih.cc
    ../pb_eih_visual_servo.cc
    ../pb_visual_servo.cc
    ../visual_servo.cc
    ../visual_servo_manager.cc
    ../single_visual_servo_manager.cc
    ../position_constraint.cc
    ../cubic_constraint.cc
    ../object_reached_termination_condition.cc
    ../termination_condition.cc
    ../vs_logger.cc
)

target_link_libraries(ecp_objectfollower_pb_eih
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)

####################################3
install(TARGETS ecp_objectfollower_pb_eih DESTINATION bin)
