if(ROBOT_SMB)
add_library(kinematicssmb
	kinematic_model_smb.cc
)

target_link_libraries(kinematicssmb kinematics)
endif(ROBOT_SMB)
