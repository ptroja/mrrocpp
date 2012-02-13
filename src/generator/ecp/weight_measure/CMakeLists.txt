add_library(ecp_g_weight_measure
	ecp_g_weight_measure.cc
)

target_link_libraries(ecp_g_weight_measure ecp)

install(TARGETS ecp_g_weight_measure DESTINATION lib)
