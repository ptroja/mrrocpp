if(ROBOT_IRP6_ON_TRACK_TFG)

add_executable(edp_irp6ot_tfg
	edp_irp6ot_tfg_effector.cc
	hi_irp6ot_tfg.cc
	int_hand_irp6ot_tfg.cc
	sg_irp6ot_tfg.cc
	 regulator_irp6ot_tfg.cc
)

target_link_libraries(edp_irp6ot_tfg kinematicsirp6ot_tfg edp hi_rydz
	${COMMON_LIBRARIES})

	
add_library(kinematicsirp6ot_tfg
	kinematic_model_irp6ot_tfg.cc
)

target_link_libraries(kinematicsirp6ot_tfg kinematic_model_irp6_tfg kinematics)
	
add_library(ecp_r_irp6ot_tfg ecp_r_irp6ot_tfg)	
	
target_link_libraries(ecp_r_irp6ot_tfg kinematicsirp6ot_tfg)

install(TARGETS edp_irp6ot_tfg DESTINATION bin)
endif(ROBOT_IRP6_ON_TRACK_TFG)
