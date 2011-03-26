# add library for visual servoing
add_library(visual_servoing
    cubic_constraint.cc
    ib_eih_visual_servo.cc
    object_reached_termination_condition.cc
    pb_eih_visual_servo.cc
    pb_sac_visual_servo.cc
    pb_visual_servo.cc
    position_constraint.cc
    single_visual_servo_manager.cc
    termination_condition.cc
    timeout_termination_condition.cc
    visual_servo_manager.cc
    visual_servo_regulator_pid.cc
    visual_servo.cc
)

# link with discode_sensor
target_link_libraries(visual_servoing
	discode_sensor
)

install(TARGETS visual_servoing DESTINATION lib)
