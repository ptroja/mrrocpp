if(QNXNTO)
	add_executable(edp_speaker
		corpus.cc
		edp_speaker_effector.cc
		speak_t.cc
	)
	
	target_link_libraries(edp_speaker edp mrrocpp
		${COMMON_LIBS})
	
	install(TARGETS edp_speaker RUNTIME DESTINATION bin)
endif(QNXNTO)
