if(ROBOT_BIRD_HAND)
add_library(kinematicsbird_hand
	kinematic_model_bird_hand.cc
	kinematic_parameters_bird_hand.cc
)

target_link_libraries(kinematicsbird_hand kinematics)
endif(ROBOT_BIRD_HAND)
