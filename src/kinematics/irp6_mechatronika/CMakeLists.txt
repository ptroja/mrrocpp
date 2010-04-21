if(ROBOT_IRP6_MECHATRONICS)
add_library(kinematicsirp6m
	kinematic_model_irp6m_5dof.cc
	kinematic_model_irp6m_with_wrist.cc
)

target_link_libraries(kinematicsirp6m kinematics)
endif(ROBOT_IRP6_MECHATRONICS)
