add_executable(ecp_gen_test
	ecp_t_gen_test.cc
	ecp_st_const_vel_gen_test.cc
	ecp_st_smooth_gen_test.cc
)

target_link_libraries(ecp_gen_test
	ecp_r_irp6p_m ecp_r_irp6ot_m
	ecp
)


add_executable(mp_gen_test
	mp_t_gen_test.cc
)

target_link_libraries(mp_gen_test mp)

install(TARGETS ecp_gen_test mp_gen_test DESTINATION bin)
