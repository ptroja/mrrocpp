add_executable(mp_ball
	mp_t_ball.cc
	mp_g_ball.cc
)

target_link_libraries(mp_ball
	kinematicsirp6ot kinematicsirp6p kinematics
	mpmain mprobots ecp_mp
	${COMMON_LIBS}
)

install(TARGETS mp_ball RUNTIME DESTINATION bin)
