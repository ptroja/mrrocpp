add_executable(ecp_sk
	ecp_g_sk.cc
	ecp_t_sk.cc
)

target_link_libraries(ecp_sk
	ecp_r_irp6p ecp_r_irp6ot
	ecp
)

add_executable(ecp_sk_mr
	ecp_g_sk.cc
	ecp_t_sk_mr.cc
)

target_link_libraries(ecp_sk_mr
	ecp_r_irp6p ecp_r_irp6ot
	ecp
)

add_executable(ecp_tfg
	ecp_g_tfg.cc
	ecp_t_tfg.cc
)

target_link_libraries(ecp_tfg
	ecp_r_irp6p_tfg ecp_r_irp6ot_tfg
	ecp
)


add_executable(mp_sk_mr
	mp_t_sk_mr.cc
)

target_link_libraries(mp_sk_mr mp)

install(TARGETS ecp_sk ecp_sk_mr ecp_tfg mp_sk_mr DESTINATION bin)
