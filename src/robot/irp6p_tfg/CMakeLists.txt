
add_executable(edp_irp6p_tfg
	edp_irp6p_tfg_effector.cc
	sg_irp6p_tfg.cc
	 regulator_irp6p_tfg.cc
)

target_link_libraries(edp_irp6p_tfg kinematicsirp6p_tfg edp hi_moxa
	${COMMON_LIBRARIES})

	
add_library(kinematicsirp6p_tfg
	kinematic_model_irp6p_tfg.cc
)

target_link_libraries(kinematicsirp6p_tfg kinematic_model_irp6_tfg kinematics)
	
add_library(ecp_r_irp6p_tfg ecp_r_irp6p_tfg.cc)	
add_library(mp_r_irp6p_tfg mp_r_irp6p_tfg.cc)	
	
target_link_libraries(ecp_r_irp6p_tfg kinematicsirp6p_tfg)

install(TARGETS edp_irp6p_tfg DESTINATION bin)
install(TARGETS kinematicsirp6p_tfg ecp_r_irp6p_tfg mp_r_irp6p_tfg DESTINATION lib)
