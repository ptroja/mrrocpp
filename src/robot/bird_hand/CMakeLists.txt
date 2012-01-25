
add_library(dp_bird_hand
	dp_bird_hand.cc
)

add_executable(edp_bird_hand
	edp_e_bird_hand.cc
	hi_bird_hand.cc
)

target_link_libraries(edp_bird_hand kinematicsbird_hand dp_bird_hand edp
	${COMMON_LIBRARIES})
	
	
	
add_library(kinematicsbird_hand
	kinematic_model_bird_hand.cc
	kinematic_parameters_bird_hand.cc
)

target_link_libraries(kinematicsbird_hand kinematics)
	
add_library(ecp_r_bird_hand ecp_r_bird_hand.cc)	
add_library(mp_r_bird_hand mp_r_bird_hand.cc)	
	
target_link_libraries(ecp_r_bird_hand kinematicsbird_hand dp_bird_hand) 
target_link_libraries(mp_r_bird_hand dp_bird_hand) 


install(TARGETS edp_bird_hand DESTINATION bin)

install(TARGETS kinematicsbird_hand ecp_r_bird_hand mp_r_bird_hand dp_bird_hand DESTINATION lib)


