add_executable(ecp_sk
	ecp_g_sk.cc
	ecp_t_sk.cc
)

target_link_libraries(ecp_sk
	ecp_r_irp6p ecp_r_irp6ot
	ecp ecp_mp ${COMMON_LIBRARIES}
)

install(TARGETS ecp_sk DESTINATION bin)
