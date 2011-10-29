add_executable(ecp_spring_contact
	ecp_g_edge_follow.cc
	ecp_t_edge_follow_mr.cc
	ecp_st_edge_follow.cc
)

target_link_libraries(ecp_spring_contact ecp_generators ecp_subtasks ecp_r_irp6ot_m ecp_r_irp6p_m)

add_executable(mp_spring_contact
	mp_t_edge_follow_mr.cc
)

target_link_libraries(mp_spring_contact mp)

install(TARGETS ecp_spring_contact mp_spring_contact DESTINATION bin)
