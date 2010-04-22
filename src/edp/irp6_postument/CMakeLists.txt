if(ROBOT_IRP6_POSTUMENT)
add_executable(edp_irp6_postument
	edp_irp6p_effector.cc
	hi_irp6p.cc
	int_hand_irp6p.cc
	sg_irp6p.cc
)

target_link_libraries(
	edp_irp6_postument
	kinematicsirp6p
	edp ${COMMON_LIBRARIES}
)

if(QNXNTO)
target_link_libraries(
	edp_irp6_postument
	ati6284
)
endif(QNXNTO)

install(TARGETS edp_irp6_postument DESTINATION bin)
endif(ROBOT_IRP6_POSTUMENT)
