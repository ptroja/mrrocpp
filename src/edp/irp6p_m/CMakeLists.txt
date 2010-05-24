if(ROBOT_IRP6_POSTUMENT_M)
add_executable(edp_irp6p_m
	edp_irp6p_m_effector.cc
	hi_irp6p_m.cc
	int_hand_irp6p_m.cc
	sg_irp6p_m.cc
	 regulator_irp6p_m.cc 
)

target_link_libraries(
	edp_irp6p_m
	kinematicsirp6p_m
	edp ${COMMON_LIBRARIES}
)

if(QNXNTO)
target_link_libraries(
	edp_irp6p_m
	ati6284
)
endif(QNXNTO)

install(TARGETS edp_irp6p_m DESTINATION bin)
endif(ROBOT_IRP6_POSTUMENT_M)
