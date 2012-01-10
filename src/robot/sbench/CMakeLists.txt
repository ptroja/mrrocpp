
add_library(dp_sbench
	dp_sbench.cc
)

add_executable(edp_sbench
	edp_e_sbench.cc
)

target_link_libraries(edp_sbench kinematicssbench dp_sbench edp
	${COMMON_LIBRARIES} ${COMEDI_LIBRARY})

		
add_library(kinematicssbench
	kinematic_model_sbench.cc
)


# Test utilities
add_executable(cpv_test_sbench cpv_test_sbench.cc)
target_link_libraries (cpv_test_sbench epos festo rt)

target_link_libraries(kinematicssbench kinematics)
	
add_library(ecp_r_sbench ecp_r_sbench.cc)	
add_library(mp_r_sbench mp_r_sbench.cc)	
	
	
target_link_libraries(ecp_r_sbench dp_sbench kinematicssbench)
target_link_libraries(mp_r_sbench dp_sbench)


install(TARGETS edp_sbench cpv_test_sbench DESTINATION bin)
install(TARGETS kinematicssbench ecp_r_sbench mp_r_sbench dp_sbench DESTINATION lib)
