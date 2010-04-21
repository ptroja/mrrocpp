add_executable(edp_irp6p_m
	edp_irp6p_m_effector.cc
	hi_irp6p_m.cc
	int_hand_irp6p_m.cc
	sg_irp6p_m.cc
)

target_link_libraries(
	edp_irp6p_m
	kinematicsirp6p
	edp ati6284
	${COMMON_LIBRARIES}
)

install(TARGETS edp_irp6p_m DESTINATION bin)
