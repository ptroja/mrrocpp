add_executable(ecp_pcbird
	ecp_t_pcbird.cc
	../../sensor/pcbird/birdclient.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_pcbird.cc
	${ECPDIR}/generator/ecp_g_cvfradia.cc
)

target_link_libraries(ecp_pcbird
	ecp_r_irp6p_m ecp_r_irp6ot_m
	ecp
)

install(TARGETS ecp_pcbird DESTINATION bin)
