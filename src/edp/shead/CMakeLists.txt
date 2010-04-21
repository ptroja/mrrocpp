if(ROBOT_SHEAD)
add_executable(edp_shead
	edp_e_shead.cc
	../common/int_hand_suplement.cc
)

target_link_libraries(edp_shead kinematicsshead edp
	${COMMON_LIBRARIES})

install(TARGETS edp_shead DESTINATION bin)
endif(ROBOT_SHEAD)
