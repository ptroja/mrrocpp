add_executable(edp_irp6p_tfg
	edp_irp6p_tfg_effector.cc
	hi_irp6p_tfg.cc
	int_hand_irp6p_tfg.cc
	sg_irp6p_tfg.cc
)

target_link_libraries(edp_irp6p_tfg kinematicsirp6p_tfg kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_irp6p_tfg RUNTIME DESTINATION bin)

