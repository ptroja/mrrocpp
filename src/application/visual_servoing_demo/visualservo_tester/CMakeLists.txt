# servo + generator sinusa
add_executable(mp_vsdemo_visualservo_tester
    mp_t_visualservo_tester.cc
)

target_link_libraries(mp_vsdemo_visualservo_tester mp)

# generator sinusa
add_executable(ecp_vsdemo_conveyor_test
    ecp_g_conveyor_sinus.cc
    ecp_t_conveyor_test.cc
)

target_link_libraries(ecp_vsdemo_conveyor_test
	ecp_robot
)

# conveyor generator: uniform acceleration
add_executable(ecp_vsdemo_conveyor_uniform_acceleration
    ecp_g_conveyor_uniform_acceleration.cc
    ecp_t_conveyor_uniform_acceleration.cc
)

target_link_libraries(ecp_vsdemo_conveyor_uniform_acceleration
    logger_client
	ecp_robot 
	${COMMON_LIBRARIES}
)


################################
install(TARGETS ecp_vsdemo_conveyor_test mp_vsdemo_visualservo_tester ecp_vsdemo_conveyor_uniform_acceleration DESTINATION bin)
