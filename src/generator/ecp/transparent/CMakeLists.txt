add_library(ecp_g_transparent
	ecp_g_transparent.cc
)

target_link_libraries(ecp_g_transparent ecp_mp_generators ecp_mp)

install(TARGETS ecp_g_transparent DESTINATION lib)
