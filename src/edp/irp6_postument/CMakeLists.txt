add_executable(edp_irp6_postument
	edp_irp6p_effector.cc
	hi_irp6p.cc
	int_hand_irp6p.cc
	sg_irp6p.cc
)

target_link_libraries(
	edp_irp6_postument
	kinematicsirp6p kinematics
	edp ati6284
	mrrocpp ${COMMON_LIBS}
)

install(TARGETS edp_irp6_postument RUNTIME DESTINATION bin)

