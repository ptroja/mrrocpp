add_executable(mp_swarm_demo_single_agent
	mp_t_swarm_demo_single_agent.cc
)

target_link_libraries(mp_swarm_demo_single_agent mp)

add_executable(ecp_spkm_swarm_demo_single_agent
	ecp_g_spkm.cc
	ecp_t_spkm.cc
)

target_link_libraries(ecp_spkm_swarm_demo_single_agent ecp_robot
)

add_executable(ecp_smb_swarm_demo_single_agent
	ecp_g_smb.cc
	ecp_t_smb.cc
)

target_link_libraries(ecp_smb_swarm_demo_single_agent
	ecp_robot
)


install(TARGETS 
ecp_spkm_swarm_demo_single_agent 
ecp_smb_swarm_demo_single_agent 
mp_swarm_demo_single_agent 
DESTINATION bin)
