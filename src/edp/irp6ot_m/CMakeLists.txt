if(ROBOT_IRP6_ON_TRACK_M)
add_executable(edp_irp6ot_m
	edp_irp6ot_m_effector.cc
	hi_irp6ot_m.cc
	int_hand_irp6ot_m.cc
	sg_irp6ot_m.cc
	 regulator_irp6ot_m.cc 
)

target_link_libraries(
	edp_irp6ot_m
	kinematicsirp6ot_m
	edp ${COMMON_LIBRARIES}
)

if(QNXNTO)
target_link_libraries(
	edp_irp6ot_m
	ati3084
)
endif(QNXNTO)

install(TARGETS edp_irp6ot_m DESTINATION bin)
endif(ROBOT_IRP6_ON_TRACK_M)
