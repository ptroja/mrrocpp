add_executable(mp_block_move
	mp_t_block_move.cc
)

target_link_libraries(mp_block_move mp)

install(TARGETS mp_block_move DESTINATION bin)