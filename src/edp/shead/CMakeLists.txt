add_executable(edp_shead
	edp_e_shead.cc
	../common/int_hand_suplement.cc
)

target_link_libraries(edp_shead kinematicsshead kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_shead DESTINATION bin)

