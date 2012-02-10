add_library(kinematicsirp6ot_tfg
	kinematic_model_irp6ot_tfg.cc
)

add_executable(edp_irp6ot_tfg
	edp_irp6ot_tfg_effector.cc
	sg_irp6ot_tfg.cc
	regulator_irp6ot_tfg.cc
)

target_link_libraries(kinematicsirp6ot_tfg kinematic_model_irp6_tfg kinematics)

target_link_libraries(edp_irp6ot_tfg kinematicsirp6ot_tfg edp hi_moxa
	${COMMON_LIBRARIES})

add_library(ecp_r_irp6ot_tfg ecp_r_irp6ot_tfg)	
add_library(mp_r_irp6ot_tfg mp_r_irp6ot_tfg)	
	
target_link_libraries(ecp_r_irp6ot_tfg kinematicsirp6ot_tfg)

install(TARGETS edp_irp6ot_tfg DESTINATION bin)
install(TARGETS kinematicsirp6ot_tfg ecp_r_irp6ot_tfg mp_r_irp6ot_tfg DESTINATION lib)

