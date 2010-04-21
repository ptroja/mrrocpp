add_executable(mp_ball
	mp_t_ball.cc
	mp_g_ball.cc
)

target_link_libraries(mp_ball
	kinematicsirp6ot kinematicsirp6p
	mpmain mprobots ecp_mp
	${COMMON_LIBRARIES}
)

install(TARGETS mp_ball DESTINATION bin)
