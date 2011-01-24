# servo + generator sinusa
add_executable(mp_visualservo_tester
    mp_t_visualservo_tester.cc
)

target_link_libraries(mp_visualservo_tester mp)

# generator sinusa
add_executable(ecp_conveyor_test
    ecp_g_conveyor_sinus.cc
    ecp_t_conveyor_test.cc
)

target_link_libraries(ecp_conveyor_test
	ecp_r_conv
	ecp ${COMMON_LIBRARIES}
)

# conveyor generator: uniform acceleration
add_executable(ecp_conveyor_uniform_acceleration
    ecp_g_conveyor_uniform_acceleration.cc
    ecp_t_conveyor_uniform_acceleration.cc
)

target_link_libraries(ecp_conveyor_uniform_acceleration
	ecp_r_conv
	ecp ${COMMON_LIBRARIES}
)


################################
install(TARGETS ecp_conveyor_test mp_visualservo_tester ecp_conveyor_uniform_acceleration DESTINATION bin)
