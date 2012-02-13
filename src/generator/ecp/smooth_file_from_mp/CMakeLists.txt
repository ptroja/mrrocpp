add_library(ecp_g_smooth_file_from_mp
	ecp_g_smooth_file_from_mp.cc
)

target_link_libraries(ecp_g_smooth_file_from_mp ecp)

install(TARGETS ecp_g_smooth_file_from_mp DESTINATION lib)
