if(ROBOT_SHEAD)
add_executable(edp_shead1
	edp_e_shead.cc
	edp_e_shead1.cc
)

target_link_libraries(edp_shead1 kinematicsshead edp
	${COMMON_LIBRARIES})

	add_executable(edp_shead2
	edp_e_shead.cc
	edp_e_shead2.cc
)

target_link_libraries(edp_shead2 kinematicsshead edp
	${COMMON_LIBRARIES})
		
add_library(kinematicsshead
	kinematic_model_shead.cc
)

target_link_libraries(kinematicsshead kinematics)
	
add_library(ecp_r_shead ecp_r_shead.cc ecp_r_shead1.cc ecp_r_shead2.cc)	
	
target_link_libraries(ecp_r_shead kinematicsshead)

install(TARGETS edp_shead1 edp_shead2 DESTINATION bin)
install(TARGETS kinematicsshead ecp_r_shead DESTINATION lib)

endif(ROBOT_SHEAD)
