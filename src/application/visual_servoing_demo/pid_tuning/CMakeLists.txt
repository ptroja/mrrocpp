# object follower - PB - EIH
add_executable(ecp_vsdemo_pid_tuning_pb_eih
    ecp_t_pid_tuning_pb_eih.cc
)

target_link_libraries(ecp_vsdemo_pid_tuning_pb_eih
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	visual_servoing
	ecp_robot
	ecp ${COMMON_LIBRARIES}
)

####################################3
install(TARGETS ecp_vsdemo_pid_tuning_pb_eih DESTINATION bin)
