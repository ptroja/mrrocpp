add_executable(ecp_teach
	ecp_t_teach.cc
)

target_link_libraries(ecp_teach
	ecp_r_irp6p ecp_r_irp6ot ecp_r_irp6m
	ecp ecp_mp ${COMMON_LIBRARIES}
)

install(TARGETS ecp_teach DESTINATION bin)
