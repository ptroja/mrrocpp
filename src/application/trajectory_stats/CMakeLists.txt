add_executable(ecp_stats
	ecp_t_stats.cc
	ecp_g_stats_generator.cc
)

target_link_libraries(ecp_stats
	ecp_robot
)

add_executable(mp_stats
	mp_t_stats.cc
)

target_link_libraries(mp_stats
	mp
)

install(TARGETS ecp_stats mp_stats DESTINATION bin)
