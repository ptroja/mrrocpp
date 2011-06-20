if(ROBOT_IRP6_POSTUMENT_M)
add_executable(edp_irp6p_m
	edp_irp6p_m_effector.cc
	sg_irp6p_m.cc
	regulator_irp6p_m.cc 
)

target_link_libraries(
	edp_irp6p_m
	kinematicsirp6p_m
	edp hi_moxa ${COMMON_LIBRARIES}
)

if(COMEDILIB_FOUND)
target_link_libraries(
        edp_irp6p_m
        ati6284KB
)
else(COMEDILIB_FOUND)
 # MacOS workaround
target_link_libraries(
        edp_irp6p_m
        # TODO zrobcie pusty czujnik
        ati3084KB
)
endif(COMEDILIB_FOUND)

add_library(kinematicsirp6p_m
	kinematic_model_calibrated_irp6p_with_wrist.cc
	kinematic_model_irp6p_5dof.cc
	kinematic_model_irp6p_jacobian_transpose_with_wrist.cc
	kinematic_model_irp6p_jacobian_with_wrist.cc
	kinematic_model_irp6p_with_wrist.cc
)

target_link_libraries(kinematicsirp6p_m kinematics)
	
add_library(ecp_r_irp6p_m ecp_r_irp6p_m.cc)
	
target_link_libraries(ecp_r_irp6p_m kinematicsirp6p_m)

install(TARGETS edp_irp6p_m DESTINATION bin)
install(TARGETS kinematicsirp6p_m ecp_r_irp6p_m DESTINATION lib)

endif(ROBOT_IRP6_POSTUMENT_M)
