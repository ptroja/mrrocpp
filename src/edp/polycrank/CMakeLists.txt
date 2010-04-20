add_executable(edp_polycrank
	edp_e_polycrank.cc
	../common/int_hand_suplement.cc
)

target_link_libraries(edp_polycrank kinematicspolycrank kinematics edp mrrocpp
	${COMMON_LIBS})

install(TARGETS edp_polycrank DESTINATION bin)

