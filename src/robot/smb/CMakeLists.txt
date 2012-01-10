add_library(dp_smb
	dp_smb.cc
)


add_library(edpsmb
	festo_and_inputs.cc
	edp_e_smb.cc
)

add_executable(edp_smb1
	edp_e_smb1.cc
)

target_link_libraries(edp_smb1 edpsmb epos festo kinematicssmb dp_smb edp
	${COMMON_LIBRARIES})
	
add_executable(edp_smb2
	edp_e_smb2.cc
)

target_link_libraries(edp_smb2 edpsmb epos festo kinematicssmb dp_smb edp
	${COMMON_LIBRARIES})
	
		
add_library(kinematicssmb
	kinematic_model_smb.cc
)

# Test utilities
add_executable(cpv_test_smb cpv_test_smb.cc)
target_link_libraries (cpv_test_smb epos festo rt)

target_link_libraries(kinematicssmb kinematics)
	
add_library(ecp_r_smb ecp_r_smb.cc ecp_r_smb1.cc ecp_r_smb2.cc)	
add_library(mp_r_smb mp_r_smb.cc mp_r_smb1.cc mp_r_smb2.cc)	
	
target_link_libraries(ecp_r_smb kinematicssmb dp_smb)
target_link_libraries(mp_r_smb dp_smb)

install(TARGETS edp_smb1 edp_smb2 cpv_test_smb DESTINATION bin)
install(TARGETS edpsmb kinematicssmb ecp_r_smb mp_r_smb dp_smb DESTINATION lib)
