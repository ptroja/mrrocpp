add_library(ecp_g_teach_in
	ecp_g_teach_in.cc
	ecp_taught_in_pose.cc
)

target_link_libraries(ecp_g_teach_in ecp)

install(TARGETS ecp_g_teach_in DESTINATION lib)
