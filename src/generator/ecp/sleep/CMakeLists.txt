add_library(ecp_g_sleep
	ecp_g_sleep.cc
)

target_link_libraries(ecp_g_sleep ecp_mp_generators ecp_mp)

install(TARGETS ecp_g_sleep DESTINATION lib)
