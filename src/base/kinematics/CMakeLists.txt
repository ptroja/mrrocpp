add_library(kinematics
	kinematic_model.cc
	kinematic_model_with_local_corrector.cc
	kinematic_model_with_tool.cc
	kinematics_manager.cc
)

target_link_libraries(kinematics mrrocpp)