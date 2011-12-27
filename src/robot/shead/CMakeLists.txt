
add_library(edpshead
	edp_e_shead.cc
)

add_executable(edp_shead1
	edp_e_shead1.cc
)

target_link_libraries(edp_shead1 edpshead kinematicsshead edp
	${COMMON_LIBRARIES} epos)

add_executable(edp_shead2
	edp_e_shead2.cc
)

target_link_libraries(edp_shead2 edpshead kinematicsshead edp
	${COMMON_LIBRARIES} epos)
		
add_library(kinematicsshead
	kinematic_model_shead.cc
)

target_link_libraries(kinematicsshead kinematics)
	
add_library(ecp_r_shead ecp_r_shead.cc ecp_r_shead1.cc ecp_r_shead2.cc)	
add_library(mp_r_shead mp_r_shead.cc mp_r_shead1.cc mp_r_shead2.cc)		
	
	
target_link_libraries(ecp_r_shead kinematicsshead)

install(TARGETS edp_shead1 edp_shead2 DESTINATION bin)
install(TARGETS edpshead kinematicsshead ecp_r_shead mp_r_shead DESTINATION lib)

