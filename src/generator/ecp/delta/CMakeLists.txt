add_library(ecp_g_delta
	ecp_g_delta.cc
)

target_link_libraries(ecp_g_delta ecp)

install(TARGETS ecp_g_delta DESTINATION lib)
