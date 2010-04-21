if(QNXNTO)
	add_executable(edp_speaker
		corpus.cc
		edp_speaker_effector.cc
		speak_t.cc
	)
	
	target_link_libraries(edp_speaker edp
		$ENV{QNX_TARGET}/x86/lib/libasound.so
		${COMMON_LIBRARIES})
	
	install(TARGETS edp_speaker DESTINATION bin)
endif(QNXNTO)
