add_executable(mp_mm_test
	#g_mm_test.cc
	#t_mm_test.cc
	mp_mm_test.cc
)

add_executable(ecp_mm_test
	#g_mm_test.cc
	t_mm_test.cc
	#mp_mm_test.cc
	../../generator/ecp/ecp_g_operator_reaction_condition.cc
)

target_link_libraries(ecp_mm_test ecp_r_irp6p_m ecp_r_irp6ot_m
	ecp_generators)

target_link_libraries(mp_mm_test mp)

install(TARGETS mp_mm_test ecp_mm_test DESTINATION bin)

