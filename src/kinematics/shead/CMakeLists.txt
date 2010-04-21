if(ROBOT_SHEAD)
add_library(kinematicsshead
	kinematic_model_shead.cc
)

target_link_libraries(kinematicsshead kinematics)
endif(ROBOT_SHEAD)
