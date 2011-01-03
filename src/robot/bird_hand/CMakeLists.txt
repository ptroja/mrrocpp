if(ROBOT_BIRD_HAND)
add_executable(edp_bird_hand
	edp_e_bird_hand.cc
	hi_bird_hand.cc
)

target_link_libraries(edp_bird_hand kinematicsbird_hand edp
	${COMMON_LIBRARIES})
	
	
	
add_library(kinematicsbird_hand
	kinematic_model_bird_hand.cc
	kinematic_parameters_bird_hand.cc
)

target_link_libraries(kinematicsbird_hand kinematics)
	
add_library(ecp_r_bird_hand ecp_r_bird_hand.cc)	
	
target_link_libraries(ecp_r_bird_hand kinematicsbird_hand)

install(TARGETS edp_bird_hand DESTINATION bin)
endif(ROBOT_BIRD_HAND)
