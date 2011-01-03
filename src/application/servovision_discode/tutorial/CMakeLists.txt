# object follower - IB - EIH
add_executable(ecp_vs_tutorial
    ecp_t_vs_tutorial.cc
    ../ib_eih_visual_servo.cc
    ../visual_servo.cc
    ../visual_servo_manager.cc
    ../single_visual_servo_manager.cc
    ../position_constraint.cc
    ../cubic_constraint.cc
    ../object_reached_termination_condition.cc
    ../timeout_termination_condition.cc
    ../termination_condition.cc
)

target_link_libraries(ecp_vs_tutorial
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)

####################################
install(TARGETS ecp_vs_tutorial DESTINATION bin)
