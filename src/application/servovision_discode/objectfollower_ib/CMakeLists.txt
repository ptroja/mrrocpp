# object follower - IB - EIH
add_executable(ecp_objectfollower_ib
    ecp_t_objectfollower_ib.cc
    ../ib_eih_visual_servo.cc
    ../visual_servo.cc
    ../visual_servo_manager.cc
    ../single_visual_servo_manager.cc
    ../position_constraint.cc
    ../cubic_constraint.cc
    ../object_reached_termination_condition.cc
    ../termination_condition.cc
)

target_link_libraries(ecp_objectfollower_ib
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)

####################################
install(TARGETS ecp_objectfollower_ib DESTINATION bin)
