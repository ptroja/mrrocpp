if(ROBOT_SPKM)

if(LibFTDI_FOUND)
add_executable(edp_spkm
	edp_e_spkm.cc
)

target_link_libraries(edp_spkm kinematicsspkm epos edp
	${COMMON_LIBRARIES})

install(TARGETS edp_spkm DESTINATION bin)
endif(LibFTDI_FOUND)

add_library(kinematicsspkm
	kinematic_model_spkm.cc
	kinematic_parameters_spkm.cc
)

target_link_libraries(kinematicsspkm kinematics)
	
add_library(ecp_r_spkm ecp_r_spkm.cc)	
	
target_link_libraries(ecp_r_spkm kinematicsspkm ecp_robot)
install(TARGETS kinematicsspkm ecp_r_spkm DESTINATION lib)

endif(ROBOT_SPKM)
