add_executable(ecp_edge_follow_mr
	ecp_g_edge_follow.cc
	ecp_t_edge_follow_mr.cc
	ecp_st_edge_follow.cc
)

target_link_libraries(ecp_edge_follow_mr ecp)


add_executable(mp_edge_follow_mr
	mp_t_edge_follow_mr.cc
)

target_link_libraries(mp_edge_follow_mr mp)

install(TARGETS ecp_edge_follow_mr mp_edge_follow_mr DESTINATION bin)
