if(ROBOT_SPKM)
add_executable(edp_spkm
	edp_e_spkm.cc
)

target_link_libraries(edp_spkm kinematicsspkm edp
	${COMMON_LIBRARIES})
	
		
add_library(kinematicsspkm
	kinematic_model_spkm.cc
	kinematic_parameters_spkm.cc
)

target_link_libraries(kinematicsspkm kinematics)
	
add_library(ecp_r_spkm ecp_r_spkm.cc)	
	
target_link_libraries(ecp_r_spkm kinematicsspkm)

install(TARGETS edp_spkm DESTINATION bin)
endif(ROBOT_SPKM)
