
add_library(dp_sbench
	dp_sbench.cc
)

add_executable(edp_sbench
	edp_e_sbench.cc
)

target_link_libraries(edp_sbench epos festo dp_sbench edp kinematics
	${COMMON_LIBRARIES} ${COMEDI_LIBRARY})

		
# Test utilities
add_executable(cpv_test_sbench cpv_test_sbench.cc)
target_link_libraries (cpv_test_sbench epos festo rt)

add_library(ecp_r_sbench ecp_r_sbench.cc)	
add_library(mp_r_sbench mp_r_sbench.cc)	
	
	
target_link_libraries(ecp_r_sbench dp_sbench kinematics)
target_link_libraries(mp_r_sbench dp_sbench)


install(TARGETS edp_sbench cpv_test_sbench DESTINATION bin)
install(TARGETS ecp_r_sbench mp_r_sbench dp_sbench DESTINATION lib)
