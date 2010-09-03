if(ROBOT_IRP6_POSTUMENT_TFG)

add_executable(edp_irp6p_tfg
	edp_irp6p_tfg_effector.cc
	hi_irp6p_tfg.cc
	int_hand_irp6p_tfg.cc
	sg_irp6p_tfg.cc
	 regulator_irp6p_tfg.cc
)

target_link_libraries(edp_irp6p_tfg kinematicsirp6p_tfg edp hi_rydz
	${COMMON_LIBRARIES})

	
add_library(kinematicsirp6p_tfg
	kinematic_model_irp6p_tfg.cc
)

target_link_libraries(kinematicsirp6p_tfg kinematic_model_irp6_tfg kinematics)
	
add_library(ecp_r_irp6p_tfg ecp_r_irp6p_tfg.cc)	
	
target_link_libraries(ecp_r_irp6p_tfg kinematicsirp6p_tfg)

install(TARGETS edp_irp6p_tfg DESTINATION bin)
endif(ROBOT_IRP6_POSTUMENT_TFG)