add_library(ecp_g_newsmooth
	ecp_g_newsmooth.cc
)

target_link_libraries(ecp_g_newsmooth ecp_smooth_base ecp_mp_generators ecp_mp)

install(TARGETS ecp_g_newsmooth DESTINATION lib)
