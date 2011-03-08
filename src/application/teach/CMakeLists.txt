add_executable(ecp_teach
	ecp_t_teach.cc
)

target_link_libraries(ecp_teach
	ecp_r_irp6p_m ecp_r_irp6ot_m
	ecp_generators
)

install(TARGETS ecp_teach DESTINATION bin)
