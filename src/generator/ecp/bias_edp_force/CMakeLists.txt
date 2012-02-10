add_library(ecp_g_bias_edp_force
	ecp_g_bias_edp_force.cc
)

target_link_libraries(ecp_g_bias_edp_force ecp)

install(TARGETS ecp_g_bias_edp_force DESTINATION lib)
