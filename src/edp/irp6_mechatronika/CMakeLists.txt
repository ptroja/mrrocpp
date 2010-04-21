if(ROBOT_IRP6_MECHATRONICS)
add_executable(edp_irp6_mechatronika
	edp_irp6m_effector.cc
	hi_irp6m.cc
	int_hand_irp6m.cc
	regulator_irp6m.cc
	sg_irp6m.cc
)
target_link_libraries(edp_irp6_mechatronika kinematicsirp6m edp
	${COMMON_LIBRARIES})

install(TARGETS edp_irp6_mechatronika DESTINATION bin)
endif(ROBOT_IRP6_MECHATRONICS)
