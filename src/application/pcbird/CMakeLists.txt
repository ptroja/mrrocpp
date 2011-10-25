add_executable(ecp_pcbird
	ecp_t_pcbird.cc
	${ECPDIR}/generator/ecp_g_cvfradia.cc
)

target_link_libraries(ecp_pcbird
	ecp_r_irp6p_m ecp_r_irp6ot_m
	pcbird_sensor
	ecp
)

install(TARGETS ecp_pcbird DESTINATION bin)
