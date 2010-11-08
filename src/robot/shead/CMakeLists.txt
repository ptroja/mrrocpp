if(ROBOT_SHEAD)
add_executable(edp_shead
	edp_e_shead.cc
)

target_link_libraries(edp_shead kinematicsshead edp
	${COMMON_LIBRARIES})
	
		
add_library(kinematicsshead
	kinematic_model_shead.cc
)

target_link_libraries(kinematicsshead kinematics)
	
add_library(ecp_r_shead ecp_r_shead.cc)	
	
target_link_libraries(ecp_r_shead kinematicsshead)

install(TARGETS edp_shead DESTINATION bin)
endif(ROBOT_SHEAD)
