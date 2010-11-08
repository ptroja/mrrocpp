if(ROBOT_CONVEYOR)

add_executable(edp_conveyor
	edp_conveyor_effector.cc
	hi_conv.cc
	regulator_conv.cc
	sg_conv.cc
)

target_link_libraries(edp_conveyor kinematicsconveyor edp hi_moxa
	${COMMON_LIBRARIES}
	)
	
add_library(kinematicsconveyor
	kinematic_model_conveyor.cc
)

target_link_libraries(kinematicsconveyor kinematics)
	
add_library(ecp_r_conv ecp_r_conv.cc)	
	
target_link_libraries(ecp_r_conv kinematicsconveyor)

install(TARGETS edp_conveyor DESTINATION bin)
endif(ROBOT_CONVEYOR)
