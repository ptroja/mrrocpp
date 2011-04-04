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
	edp hi_rydz ${COMMON_LIBRARIES}
)


target_link_libraries(
	edp_irp6ot_m
	ati3084KB
)

	
add_library(kinematicsirp6ot_m
	kinematic_model_calibrated_irp6ot_with_wrist.cc
	kinematic_model_irp6ot_with_track.cc
	kinematic_model_irp6ot_with_wrist.cc
)

target_link_libraries(kinematicsirp6ot_m kinematics)
	
add_library(ecp_r_irp6ot_m ecp_r_irp6ot_m.cc)
	
target_link_libraries(ecp_r_irp6ot_m kinematicsirp6ot_m)

install(TARGETS edp_irp6ot_m DESTINATION bin)
install(TARGETS kinematicsirp6ot_m ecp_r_irp6ot_m DESTINATION lib)

endif(ROBOT_IRP6_ON_TRACK_M)
