add_executable(mp_bird_hand_test
	mp_t_bird_hand_test.cc
)

target_link_libraries(mp_bird_hand_test mp)

add_executable(ecp_bird_hand_test
	ecp_g_bird_hand_test.cc
	ecp_t_bird_hand_test.cc
)

target_link_libraries(ecp_bird_hand_test
	 ecp
)

install(TARGETS ecp_bird_hand_test mp_bird_hand_test DESTINATION bin)
