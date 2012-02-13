add_library(ecp_g_get_position
	ecp_g_get_position.cc
)

target_link_libraries(ecp_g_get_position ecp)

install(TARGETS ecp_g_get_position DESTINATION lib)
