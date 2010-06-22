add_executable(ecp_cvfradia
	ecp_t_cvfradia.cc
	${ECPDIR}/generator/ecp_g_cvfradia.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_cvfradia.cc
)

target_link_libraries(ecp_cvfradia
	ecp_r_irp6ot_m ecp_r_irp6p_m
	ecp
)

install(TARGETS ecp_cvfradia DESTINATION bin)
