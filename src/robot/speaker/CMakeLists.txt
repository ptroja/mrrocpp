if(ROBOT_SPEAKER AND QNXNTO)
	add_executable(edp_speaker
		corpus.cc
		edp_speaker_effector.cc
		speak_t.cc
	)
	
	target_link_libraries(edp_speaker edp
		$ENV{QNX_TARGET}/x86/lib/libasound.so
		kinematics
		${COMMON_LIBRARIES})
	
	add_library(ecp_r_speaker ecp_r_speaker.cc)

	add_executable(ecp_speaker
		ecp_t_s.cc
		ecp_g_speak.cc
	)

	target_link_libraries(ecp_speaker ecp_r_speaker ecp)

	install(TARGETS edp_speaker DESTINATION bin)
	
endif(ROBOT_SPEAKER AND QNXNTO)
