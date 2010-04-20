add_executable(edp_spkm
	edp_e_spkm.cc
	../common/int_hand_suplement.cc
)

target_link_libraries(edp_spkm kinematicsspkm kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_spkm DESTINATION bin)

