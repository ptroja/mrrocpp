add_executable(edp_irp6_on_track
	edp_irp6ot_effector.cc
	hi_irp6ot.cc
	int_hand_irp6ot.cc
	sg_irp6ot.cc
)

target_link_libraries(edp_irp6_on_track kinematicsirp6ot kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_irp6_on_track RUNTIME DESTINATION bin)
