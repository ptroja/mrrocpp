if(ROBOT_SPEAKER)
add_ecp_robot(speaker)

add_executable(ecp_speaker
	task/ecp_t_s.cc
	generator/ecp_g_speak.cc
)

target_link_libraries(ecp_speaker ecp_r_speaker ecp)

install(TARGETS ecp_speaker DESTINATION bin)
endif(ROBOT_SPEAKER)
