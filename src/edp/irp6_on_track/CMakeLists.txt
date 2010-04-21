add_executable(edp_irp6_on_track
	edp_irp6ot_effector.cc
	hi_irp6ot.cc
	int_hand_irp6ot.cc
	sg_irp6ot.cc
)

target_link_libraries(
	edp_irp6_on_track
	kinematicsirp6ot
	edp ati3084
	${COMMON_LIBRARIES}
)

install(TARGETS edp_irp6_on_track DESTINATION bin)
