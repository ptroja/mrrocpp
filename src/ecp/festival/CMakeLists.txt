add_executable(ecp_festival
	task/ecp_t_festival.cc
	generator/ecp_g_festival.cc
)

target_link_libraries(ecp_festival
	ecp ecp_mp ${COMMON_LIBS})

install(TARGETS ecp_festival DESTINATION bin)
