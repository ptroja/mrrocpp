

add_executable(edp_sarkofag
	edp_e_sarkofag.cc
	hi_sarkofag.cc
	int_hand_sarkofag.cc
	sg_sarkofag.cc
	 regulator_sarkofag.cc
)

target_link_libraries(edp_sarkofag kinematicssarkofag edp
	${COMMON_LIBRARIES})

	
add_library(kinematicssarkofag
	kinematic_model_sarkofag.cc
)

target_link_libraries(kinematicssarkofag kinematics)
	
add_library(ecp_r_sarkofag ecp_r_sarkofag)	
	
target_link_libraries(ecp_r_sarkofag kinematicssarkofag)

install(TARGETS edp_sarkofag DESTINATION bin)

