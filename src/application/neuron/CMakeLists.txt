add_executable(ecp_neuron
	ecp_t_neuron.cc
	neuron_sensor.cc
	ecp_g_neuron_generator.cc
)

target_link_libraries(ecp_neuron
	ecp_r_irp6ot_m
	ecp
)

add_executable(mp_neuron
	mp_t_neuron.cc
)

target_link_libraries(mp_neuron
	mp
)

install(TARGETS ecp_neuron mp_neuron DESTINATION bin)
