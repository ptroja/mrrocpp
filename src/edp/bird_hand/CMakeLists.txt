if(ROBOT_BIRD_HAND)
add_executable(edp_bird_hand
	edp_e_bird_hand.cc
	../common/int_hand_suplement.cc
	bird_hand.cpp
)

target_link_libraries(edp_bird_hand kinematicsbird_hand edp
	${COMMON_LIBRARIES})

install(TARGETS edp_bird_hand DESTINATION bin)
endif(ROBOT_BIRD_HAND)
