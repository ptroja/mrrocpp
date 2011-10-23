add_executable(mp_swarm_demo_single_agent
	mp_t_swarm_demo_single_agent.cc
)

target_link_libraries(mp_swarm_demo_single_agent mp)

add_executable(ecp_swarm_demo_single_agent
	ecp_g_spkm.cc
	ecp_t_spkm.cc
)

target_link_libraries(ecp_swarm_demo_single_agent ecp_r_spkm ecp 
)

#add_executable(ecp_smb
#	ecp_g_spkm.cc
#	ecp_g_smb.cc
#	ecp_t_smb.cc
#)

#target_link_libraries(ecp_smb
#	ecp_generators
#)

#add_executable(ecp_shead
#	ecp_g_shead.cc
#	ecp_t_shead.cc
#)

#target_link_libraries(ecp_shead
#	ecp_generators
#)

#install(TARGETS ecp_spkm ecp_smb ecp_shead mp_swarmitfix DESTINATION bin)
install(TARGETS ecp_swarm_demo_single_agent mp_swarm_demo_single_agent DESTINATION bin)
