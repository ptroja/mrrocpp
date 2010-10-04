if(ROBOT_POLYCRANK)

add_executable(edp_polycrank
	edp_e_polycrank.cc
)

target_link_libraries(edp_polycrank kinematicspolycrank edp
	${COMMON_LIBRARIES})
	
add_library(kinematicspolycrank
	kinematic_model_polycrank.cc
)

target_link_libraries(kinematicspolycrank kinematics)
	
add_library(ecp_r_polycrank ecp_r_polycrank.cc)	
	
target_link_libraries(ecp_r_polycrank kinematicspolycrank)

install(TARGETS edp_polycrank DESTINATION bin)
endif(ROBOT_POLYCRANK)
