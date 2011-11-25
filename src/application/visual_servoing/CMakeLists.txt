# add library for visual servoing
add_library(visual_servoing
    visual_servo_manager.cc
    single_visual_servo_manager.cc
    
    visual_servo.cc
    ib_eih_visual_servo.cc
    pb_visual_servo.cc
    pb_eih_visual_servo.cc
    pb_sac_visual_servo.cc
    
    visual_servo_regulator.cc
    visual_servo_regulator_p.cc
    visual_servo_regulator_pid.cc
    
    position_constraint.cc
    cubic_constraint.cc
    
    termination_condition.cc
    object_reached_termination_condition.cc
    timeout_termination_condition.cc
)

# link with discode_sensor
target_link_libraries(visual_servoing
	discode_sensor
	logger_client
	${COMMON_LIBRARIES}
)

install(TARGETS visual_servoing DESTINATION lib)

add_subdirectory(logger_server)
add_subdirectory(logger_test)
