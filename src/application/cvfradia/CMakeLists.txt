add_executable(ecp_cvfradia
	ecp_t_cvfradia.cc
	${ECPDIR}/common/generator/ecp_g_cvfradia.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_cvfradia.cc
)

target_link_libraries(ecp_cvfradia
	ecp_r_irp6ot ecp_r_irp6p
	kinematicsirp6ot kinematicsirp6p kinematics
	ecp ecp_mp
	${COMMON_LIBS}
)

install(TARGETS ecp_cvfradia DESTINATION bin)
