add_executable(edp_irp6p_m
	edp_irp6p_m_effector.cc
	hi_irp6p_m.cc
	int_hand_irp6p_m.cc
	sg_irp6p_m.cc
)

target_link_libraries(
	edp_irp6p_m
	kinematicsirp6p kinematics
	edp ati6284
	mrrocpp ${COMMON_LIBS}
)

install(TARGETS edp_irp6p_m RUNTIME DESTINATION bin)
