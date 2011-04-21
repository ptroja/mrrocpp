add_executable(mp_mm_test
	mp_mm_test.cc
)

add_executable(ecp_mm_test
	t_mm_test.cc
	../edge_follow/ecp_st_edge_follow.cc
    ../edge_follow/ecp_g_edge_follow.cc
    g_mm_test.cc
)

target_link_libraries(ecp_mm_test 
    ecp_r_irp6p_m 
    ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
	visual_servoing
	ecp_generators
	ecp_subtasks
)

target_link_libraries(mp_mm_test mp)


install(TARGETS mp_mm_test ecp_mm_test DESTINATION bin)

