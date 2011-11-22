add_executable(ecp_teach
	ecp_t_teach.cc
)

target_link_libraries(ecp_teach
	ecp
)

install(TARGETS ecp_teach DESTINATION bin)
