add_library(ecp_g_spline
	ecp_g_spline.cc
)

target_link_libraries(ecp_g_spline ecp)

install(TARGETS ecp_g_spline DESTINATION lib)
