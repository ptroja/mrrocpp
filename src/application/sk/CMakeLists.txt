add_executable(ecp_sk
	ecp_g_sk.cc
	ecp_t_sk.cc
)

target_link_libraries(ecp_sk
	ecp_r_irp6p_m ecp_r_irp6ot_m
	ecp
)

add_executable(ecp_sk_mr
	ecp_g_sk.cc
	ecp_t_sk_mr.cc
)

target_link_libraries(ecp_sk_mr
	ecp_r_irp6p_m ecp_r_irp6ot_m
	ecp
)


add_executable(mp_sk_mr
	mp_t_sk_mr.cc
)

target_link_libraries(mp_sk_mr mp)

install(TARGETS ecp_sk ecp_sk_mr mp_sk_mr DESTINATION bin)
