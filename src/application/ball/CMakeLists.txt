
add_executable(mp_ball
	mp_t_ball.cc
	mp_g_ball.cc
)

target_link_libraries(mp_ball mp)

install(TARGETS mp_ball DESTINATION bin)
