if(ROBOT_SMB)
add_executable(edp_smb1
	edp_e_smb1.cc
	festo_and_inputs.cc
	edp_e_smb.cc
)

target_link_libraries(edp_smb1 epos festo kinematicssmb edp
	${COMMON_LIBRARIES})
	
add_executable(edp_smb2
	edp_e_smb2.cc
	festo_and_inputs.cc
	edp_e_smb.cc
)

target_link_libraries(edp_smb2 epos festo kinematicssmb edp
	${COMMON_LIBRARIES})
	
		
add_library(kinematicssmb
	kinematic_model_smb.cc
)

target_link_libraries(kinematicssmb kinematics)
	
add_library(ecp_r_smb ecp_r_smb.cc ecp_r_smb1.cc ecp_r_smb2.cc)	
	
target_link_libraries(ecp_r_smb kinematicssmb)

install(TARGETS edp_smb1 edp_smb2 DESTINATION bin)
install(TARGETS kinematicssmb ecp_r_smb DESTINATION lib)

endif(ROBOT_SMB)
