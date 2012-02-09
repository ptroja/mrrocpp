add_executable(mp_swarm_demo_irp6
	mp_t_swarm_demo_irp6.cc
)

target_link_libraries(mp_swarm_demo_irp6 mp)

install(TARGETS mp_swarm_demo_irp6 DESTINATION bin)
