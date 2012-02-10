

add_executable(edp_conveyor
	edp_conveyor_effector.cc
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
add_library(mp_r_conv mp_r_conveyor.cc)	
	
target_link_libraries(ecp_r_conv kinematicsconveyor)

install(TARGETS edp_conveyor DESTINATION bin)
install(TARGETS ecp_r_conv mp_r_conv kinematicsconveyor DESTINATION lib)
