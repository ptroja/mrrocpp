add_library(kinematicsspkm
	kinematic_model_spkm.cc
	kinematic_parameters_spkm.cc
)

target_link_libraries(kinematicsspkm kinematics)
