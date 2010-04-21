if(ROBOT_CONVEYOR)
add_library(kinematicsconveyor
	kinematic_model_conveyor.cc
)

target_link_libraries(kinematicsconveyor kinematics)
endif(ROBOT_CONVEYOR)
