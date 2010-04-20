add_executable(ecp_graspit
	ecp_mp_tr_graspit.cc
	ecp_t_graspit.cc
)

target_link_libraries(ecp_graspit
	ecp_r_irp6ot ecp_r_irp6p
	kinematicsirp6ot kinematicsirp6p kinematics
	ecp ecp_mp ${COMMON_LIBS}
)

install(TARGETS ecp_graspit DESTINATION bin)
