add_executable(ecp_block_move
	ecp_t_block_move.cc
)

target_link_libraries(ecp_block_move ecp_robot)

add_executable(mp_block_move
	mp_t_block_move.cc
)

target_link_libraries(mp_block_move mp)

install(TARGETS ecp_block_move mp_block_move DESTINATION bin)