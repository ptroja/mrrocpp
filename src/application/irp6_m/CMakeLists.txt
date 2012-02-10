
add_executable(ecp_irp6_m
	ecp_t_irp6_m.cc
)


target_link_libraries(ecp_irp6_m
    ecp_robot ecp_generators
	)

install(TARGETS
	ecp_irp6_m
	DESTINATION bin)
