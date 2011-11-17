if(ROBOT_SBENCH)
add_executable(edp_sbench
	edp_e_sbench.cc
)

target_link_libraries(edp_sbench kinematicssbench edp
	${COMMON_LIBRARIES} ${COMEDI_LIBRARY})

		
add_library(kinematicssbench
	kinematic_model_sbench.cc
)

target_link_libraries(kinematicssbench kinematics)
	
add_library(ecp_r_sbench ecp_r_sbench.cc)	
	
target_link_libraries(ecp_r_sbench kinematicssbench)

install(TARGETS edp_sbench DESTINATION bin)
install(TARGETS kinematicssbench ecp_r_sbench DESTINATION lib)

endif(ROBOT_SBENCH)
