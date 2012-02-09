
add_executable(ecp_irp6m_common
	ecp_t_irp6m_common.cc
)


target_link_libraries(ecp_irp6m_common
    ecp_robot
	)

install(TARGETS
	ecp_irp6m_common
	DESTINATION bin)
