if(ROBOT_IRP6_MECHATRONICS)


add_executable(edp_irp6_mechatronika
	edp_irp6m_effector.cc
	hi_irp6m.cc
	int_hand_irp6m.cc
	regulator_irp6m.cc
	sg_irp6m.cc
)
target_link_libraries(edp_irp6_mechatronika kinematicsirp6m edp hi_rydz
	${COMMON_LIBRARIES})




	
add_library(kinematicsirp6m
	kinematic_model_irp6m_5dof.cc
	kinematic_model_irp6m_with_wrist.cc
)

target_link_libraries(kinematicsirp6m kinematics)
	
add_library(ecp_r_irp6m ecp_r_irp6m)	
	
target_link_libraries(ecp_r_irp6m kinematicsirp6m)

install(TARGETS edp_irp6_mechatronika DESTINATION bin)

endif(ROBOT_IRP6_MECHATRONICS)
