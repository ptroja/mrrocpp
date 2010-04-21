add_executable(edp_irp6ot_m
	edp_irp6ot_m_effector.cc
	hi_irp6ot_m.cc
	int_hand_irp6ot_m.cc
	sg_irp6ot_m.cc
)

target_link_libraries(
	edp_irp6ot_m
	kinematicsirp6ot
	edp ati3084
	${COMMON_LIBRARIES}
)

install(TARGETS edp_irp6ot_m DESTINATION bin)
