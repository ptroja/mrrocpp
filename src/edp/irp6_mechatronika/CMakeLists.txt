add_executable(edp_irp6_mechatronika
	edp_irp6m_effector.cc
	hi_irp6m.cc
	int_hand_irp6m.cc
	regulator_irp6m.cc
	sg_irp6m.cc
)
target_link_libraries(edp_irp6_mechatronika kinematicsirp6m kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_irp6_mechatronika DESTINATION bin)
