if(ROBOT_SMB)
add_executable(edp_smb
	edp_e_smb.cc
)

target_link_libraries(edp_smb kinematicssmb edp
	${COMMON_LIBRARIES})
	
		
add_library(kinematicssmb
	kinematic_model_smb.cc
)

target_link_libraries(kinematicssmb kinematics)
	
add_library(ecp_r_smb ecp_r_smb.cc)	
	
target_link_libraries(ecp_r_smb kinematicssmb)

install(TARGETS edp_smb DESTINATION bin)
endif(ROBOT_SMB)
