add_executable(ecp_tran
	../common/task/ecp_t_tran.cc
	../common/generator/ecp_g_transparent.cc
)

target_link_libraries(ecp_tran
	ecp_r_irp6ot ecp_r_irp6p ecp_r_irp6m
	ecp_r_polycrank ecp_r_conv ecp_r_speaker
	kinematicsirp6ot
	kinematicsirp6p
	kinematics
	ecp ecp_mp ${COMMON_LIBS}
)

install(TARGETS ecp_tran RUNTIME DESTINATION bin)
