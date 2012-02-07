

add_executable(edp_irp6ot_m
	edp_irp6ot_m_effector.cc
	sg_irp6ot_m.cc
	regulator_irp6ot_m.cc 
)

target_link_libraries(
	edp_irp6ot_m
	kinematicsirp6ot_m
	edp hi_moxa ${COMMON_LIBRARIES}
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
add_library(mp_r_irp6ot_m mp_r_irp6ot_m.cc)
	
target_link_libraries(ecp_r_irp6ot_m kinematicsirp6ot_m)

install(TARGETS edp_irp6ot_m DESTINATION bin)
install(TARGETS kinematicsirp6ot_m ecp_r_irp6ot_m mp_r_irp6ot_m DESTINATION lib)

