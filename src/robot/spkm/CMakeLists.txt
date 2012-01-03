if(LibFTDI_FOUND)

add_library(dp_spkm
	dp_spkm.cc
)

add_library(edpspkm
edp_e_spkm.cc
)

add_executable(edp_spkm1
	edp_e_spkm1.cc
	kinematic_parameters_spkm1.cpp
)

target_link_libraries(edp_spkm1 edpspkm dp_spkm kinematicsspkm epos edp
	${COMMON_LIBRARIES})
	
	
add_executable(edp_spkm2
	edp_e_spkm2.cc
	kinematic_parameters_spkm2.cpp
)

target_link_libraries(edp_spkm2 edpspkm dp_spkm kinematicsspkm epos edp
	${COMMON_LIBRARIES})	

install(TARGETS edp_spkm1 edp_spkm2 DESTINATION bin)
endif(LibFTDI_FOUND)

add_library(kinematicsspkm
	kinematic_model_spkm.cc
)

target_link_libraries(kinematicsspkm kinematics)
	
add_library(ecp_r_spkm ecp_r_spkm.cc ecp_r_spkm1.cc ecp_r_spkm2.cc kinematic_parameters_spkm1.cpp kinematic_parameters_spkm2.cpp)	
add_library(mp_r_spkm mp_r_spkm.cc mp_r_spkm1.cc mp_r_spkm2.cc kinematic_parameters_spkm1.cpp kinematic_parameters_spkm2.cpp)	

target_link_libraries(ecp_r_spkm kinematicsspkm dp_spkm)
target_link_libraries(mp_r_spkm dp_spkm)
	
install(TARGETS edpspkm kinematicsspkm ecp_r_spkm mp_r_spkm dp_spkm DESTINATION lib)
