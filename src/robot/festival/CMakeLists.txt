add_executable(ecp_festival
	ecp_t_festival.cc
	ecp_g_festival.cc
)

target_link_libraries(ecp_festival ecp_generators)

install(TARGETS ecp_festival DESTINATION bin)
