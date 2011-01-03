if(QNXNTO)
add_executable(vsp_time_int
	vsp_time_sensor.cc
	../../base/vsp/vsp_m_int.cc ../../base/vsp/vsp_sensor_interface.cc
)

target_link_libraries(vsp_time_int
	${COMMON_LIBRARIES}
)

add_executable(vsp_time_nint
	vsp_time_sensor.cc
	../../base/vsp/vsp_m_nint.cc ../../base/vsp/vsp_sensor_interface.cc
)

target_link_libraries(vsp_time_nint
	${COMMON_LIBRARIES}
)

#add_executable(vsp_time_int_nw
#	vsp_time_sensor.cc
#	../../base/vsp/vsp_m_int_nw.cc ../../base/vsp/vsp_sensor_interface.cc
#)

#target_link_libraries(vsp_time_int_nw
#	${COMMON_LIBRARIES}
#)

add_executable(ecp_time
	ecp_g_time.cc ecp_mp_s_time.cc ecp_t_time.cc
)

target_link_libraries(ecp_time ecp_r_irp6ot_m ecp_r_irp6p_m ecp)

install(TARGETS vsp_time_int DESTINATION bin)
endif(QNXNTO)
