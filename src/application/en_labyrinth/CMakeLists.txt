add_executable(mp_en_labyrinth
	mp_en_labyrinth.cc
)

add_executable(ecp_en_labyrinth
	ecp_en_labyrinth.cc
    ecp_g_en_labyrinth.cc
)

target_link_libraries(ecp_en_labyrinth 
    ecp_r_irp6p_m 
    ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
	visual_servoing
	ecp_robot
	discode_sensor
)

target_link_libraries(mp_en_labyrinth mp discode_sensor)

install(TARGETS mp_en_labyrinth ecp_en_labyrinth DESTINATION bin)


