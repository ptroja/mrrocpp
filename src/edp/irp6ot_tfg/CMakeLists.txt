if(ROBOT_IRP6_ON_TRACK_TFG)
add_executable(edp_irp6ot_tfg
	edp_irp6ot_tfg_effector.cc
	hi_irp6ot_tfg.cc
	int_hand_irp6ot_tfg.cc
	sg_irp6ot_tfg.cc
	 regulator_irp6ot_tfg.cc
)

target_link_libraries(edp_irp6ot_tfg kinematicsirp6ot_tfg edp
	${COMMON_LIBRARIES})

install(TARGETS edp_irp6ot_tfg DESTINATION bin)
endif(ROBOT_IRP6_ON_TRACK_TFG)
