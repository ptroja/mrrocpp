add_executable(edp_conveyor
	edp_conveyor_effector.cc
	hi_conv.cc
	int_hand_conv.cc
	regulator_conv.cc
	sg_conv.cc
)
target_link_libraries(edp_conveyor kinematicsconveyor kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_conveyor RUNTIME DESTINATION bin)
