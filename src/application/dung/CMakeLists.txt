add_executable(ecp_dung
	ecp_g_dung.cc
	ecp_t_dung.cc
)

target_link_libraries(ecp_dung
	ecp_r_irp6ot ecp_r_irp6p
	kinematicsirp6ot kinematicsirp6p
	ecp ecp_mp ${COMMON_LIBRARIES}
)

install(TARGETS ecp_dung DESTINATION bin)
