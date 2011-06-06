add_library(mp_generator_base
	mp_generator.cc
	mp_g_delay_ms_condition.cc
	mp_g_wait_for_task_termination.cc
	mp_g_set_next_ecps_state.cc
	mp_g_send_end_motion_to_ecps.cc
	mp_g_continously_coordinated.cc
)

install(TARGETS mp_generator_base DESTINATION lib)
