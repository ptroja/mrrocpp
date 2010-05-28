# object follower - PB - SAC
add_executable(ecp_objectfollower_pb_sac
    objectfollower_pb_sac/ecp_t_objectfollower_pb_sac.cpp
    pb_sac_visual_servo.cc
    visual_servo.cc
    visual_servo_manager.cc
    simple_visual_servo_manager.cc
    position_constraint.cc
    cubic_constraint.cc
    logger.cc
)

target_link_libraries(ecp_objectfollower_pb_sac
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)

# object follower - PB - EIH
add_executable(ecp_objectfollower_pb
    objectfollower_pb/ecp_t_objectfollower_pb.cc
    pb_eih_visual_servo.cc
    visual_servo.cc
    visual_servo_manager.cc
    simple_visual_servo_manager.cc
    position_constraint.cc
    cubic_constraint.cc
    object_reached_termination_condition.cc
    termination_condition.cc
    logger.cc
)

target_link_libraries(ecp_objectfollower_pb
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)

# object follower - IB - EIH
add_executable(ecp_objectfollower_ib
    objectfollower_ib/ecp_t_objectfollower_ib.cc
    ib_eih_visual_servo.cc
    visual_servo.cc
    visual_servo_manager.cc
    simple_visual_servo_manager.cc
    position_constraint.cc
    cubic_constraint.cc
    object_reached_termination_condition.cc
    termination_condition.cc
    logger.cc
)

target_link_libraries(ecp_objectfollower_ib
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)

# robienie zdjec obiektu z zadanych pozycji

add_executable(ecp_image_capture
    image_capture/capture_task.cc
    logger.cc
)

target_link_libraries(ecp_image_capture
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)


# conveyor test

####################################3
install(TARGETS ecp_objectfollower_ib ecp_objectfollower_pb ecp_objectfollower_pb_sac ecp_image_capture DESTINATION bin)


add_subdirectory(visualservo_tester)
