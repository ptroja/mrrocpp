if(ROBOT_POLYCRANK)

add_executable(edp_polycrank
	edp_e_polycrank.cc
	hi_polycrank.cc
	regulator_polycrank.cc
	sg_polycrank.cc
)

target_link_libraries(edp_polycrank kinematicspolycrank edp hi_moxa
	${COMMON_LIBRARIES}
	)
	
add_library(kinematicspolycrank
	kinematic_model_polycrank.cc
)

target_link_libraries(kinematicspolycrank kinematics)
	
add_library(ecp_r_polycrank ecp_r_polycrank.cc)	
	
target_link_libraries(ecp_r_polycrank kinematicspolycrank)

install(TARGETS edp_polycrank DESTINATION bin)
install(TARGETS kinematicspolycrank ecp_r_polycrank DESTINATION lib)

endif(ROBOT_POLYCRANK)
