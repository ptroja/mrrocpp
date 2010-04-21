if(ROBOT_POLYCRANK)
add_library(kinematicspolycrank
	kinematic_model_polycrank.cc
)

target_link_libraries(kinematicspolycrank kinematics)
endif(ROBOT_POLYCRANK)
