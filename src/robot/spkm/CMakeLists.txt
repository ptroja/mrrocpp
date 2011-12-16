if(ROBOT_SPKM)

if(LibFTDI_FOUND)


add_library(edpspkm
edp_e_spkm.cc
)

add_executable(edp_spkm1
	edp_e_spkm1.cc
)

target_link_libraries(edp_spkm1 edpspkm kinematicsspkm epos edp
	${COMMON_LIBRARIES})
	
	
add_executable(edp_spkm2
	edp_e_spkm2.cc
)

target_link_libraries(edp_spkm2 edpspkm kinematicsspkm epos edp
	${COMMON_LIBRARIES})	

install(TARGETS edp_spkm1 edp_spkm2 DESTINATION bin)
endif(LibFTDI_FOUND)

add_library(kinematicsspkm
	kinematic_model_spkm.cc
	kinematic_parameters_spkm.cc
)

add_executable(xmltest xmltest.cc)
target_link_libraries(xmltest mrrocpp ${COMMON_LIBRARIES}) 

target_link_libraries(kinematicsspkm kinematics)
	
add_library(ecp_r_spkm ecp_r_spkm.cc ecp_r_spkm1.cc ecp_r_spkm2.cc)	
target_link_libraries(ecp_r_spkm kinematicsspkm)
	
install(TARGETS edpspkm kinematicsspkm ecp_r_spkm DESTINATION lib)

endif(ROBOT_SPKM)
