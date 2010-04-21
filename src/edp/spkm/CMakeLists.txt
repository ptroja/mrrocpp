if(ROBOT_SPKM)
add_executable(edp_spkm
	edp_e_spkm.cc
	../common/int_hand_suplement.cc
)

target_link_libraries(edp_spkm kinematicsspkm edp
	${COMMON_LIBRARIES})

install(TARGETS edp_spkm DESTINATION bin)
endif(ROBOT_SPKM)
