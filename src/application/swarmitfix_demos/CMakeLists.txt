## Last Warsaw demo. 
add_executable(mp_swarmitfix_demo_agent1_warsaw 
	mp_t_swarmitfix_demo_base.cc
	mp_t_swarmitfix_demo_agent1_warsaw.cpp
)
target_link_libraries(mp_swarmitfix_demo_agent1_warsaw mp)

## SMB test program. 
add_executable(mp_swarmitfix_smb_test 
	mp_t_swarmitfix_demo_base.cc
	mp_t_swarmitfix_smb_test.cpp
)
target_link_libraries(mp_swarmitfix_smb_test mp)


## SMB *transparent* process. 
add_executable(ecp_smb_swarm_demo_single_agent
	ecp_g_smb.cc
	ecp_t_smb.cc
)
target_link_libraries(ecp_smb_swarm_demo_single_agent
	ecp_robot
)

## SPKM *transparent* process. 
add_executable(ecp_spkm_swarm_demo_single_agent
	ecp_g_spkm.cc
	ecp_t_spkm.cc
)
target_link_libraries(ecp_spkm_swarm_demo_single_agent ecp_robot
)

## SHEAD *transparent* process. 
add_executable(ecp_shead_swarm_demo_single_agent
	ecp_g_shead.cc
	ecp_t_shead.cc
)
target_link_libraries(ecp_shead_swarm_demo_single_agent
	ecp_robot
)


install(TARGETS 
ecp_spkm_swarm_demo_single_agent 
ecp_smb_swarm_demo_single_agent 
ecp_shead_swarm_demo_single_agent
mp_swarmitfix_demo_agent1_warsaw
mp_swarmitfix_smb_test
DESTINATION bin)
