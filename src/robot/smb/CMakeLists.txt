if(ROBOT_SMB)
add_executable(edp_smb1
	edp_e_smb1.cc
	edp_e_smb.cc
)

target_link_libraries(edp_smb1 kinematicssmb edp
	${COMMON_LIBRARIES})
	
add_executable(edp_smb2
	edp_e_smb2.cc
	edp_e_smb.cc
)

target_link_libraries(edp_smb2 kinematicssmb edp
	${COMMON_LIBRARIES})
	
		
add_library(kinematicssmb
	kinematic_model_smb.cc
)

target_link_libraries(kinematicssmb kinematics)
	
add_library(ecp_r_smb ecp_r_smb.cc)	
	
target_link_libraries(ecp_r_smb kinematicssmb)

install(TARGETS edp_smb1 edp_smb2 DESTINATION bin)
install(TARGETS kinematicssmb ecp_r_smb DESTINATION lib)

endif(ROBOT_SMB)
