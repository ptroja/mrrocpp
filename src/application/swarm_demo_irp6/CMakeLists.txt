add_executable(ecp_swarm_demo_irp6
	ecp_t_swarm_demo_irp6.cc
)

target_link_libraries(ecp_swarm_demo_irp6 ecp_r_irp6ot_m ecp_r_irp6p_m ecp_generators ecp_subtasks ecp)

add_executable(mp_swarm_demo_irp6
	mp_t_swarm_demo_irp6.cc
)

target_link_libraries(mp_swarm_demo_irp6 mp)

install(TARGETS ecp_swarm_demo_irp6 mp_swarm_demo_irp6 DESTINATION bin)
