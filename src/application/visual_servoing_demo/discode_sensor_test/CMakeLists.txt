add_executable(ecp_discode_sensor_test
	ecp_g_discode_sensor_test.cc
    ecp_t_discode_sensor_test.cc
)

target_link_libraries(ecp_discode_sensor_test
	ecp_r_irp6p_m
	discode_sensor
	ecp ${COMMON_LIBRARIES}
)

################################
install(TARGETS ecp_discode_sensor_test DESTINATION bin)
