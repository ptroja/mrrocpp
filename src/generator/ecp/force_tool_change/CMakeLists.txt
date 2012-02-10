add_library(ecp_g_force_tool_change
	ecp_g_force_tool_change.cc
)

target_link_libraries(ecp_g_force_tool_change ecp)

install(TARGETS ecp_g_force_tool_change DESTINATION lib)
