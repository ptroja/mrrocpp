
add_executable(edp_irp6ot_tfg
	edp_irp6ot_tfg_effector.cc
	hi_irp6ot_tfg.cc
	int_hand_irp6ot_tfg.cc
	sg_irp6ot_tfg.cc
)

target_link_libraries(edp_irp6ot_tfg kinematicsirp6ot_tfg kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_irp6ot_tfg DESTINATION bin)
