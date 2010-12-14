
add_executable(ecp_pb_sac_calibration
    ecp_t_pb_sac_calibration.cc
    pb_sac_calibration.cc
    ../pb_visual_servo.cc
    ../visual_servo.cc
    ../visual_servo_manager.cc
    ../single_visual_servo_manager.cc
    ../termination_condition.cc
    ../../../sensor/discode/discode_sensor.cc
)

target_link_libraries(ecp_pb_sac_calibration
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)


################################
install(TARGETS ecp_pb_sac_calibration DESTINATION bin)
