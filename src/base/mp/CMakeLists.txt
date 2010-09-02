add_library(mp
    MP_main_error.cc
	mp_task.cc
	mp_robot.cc
	mp_m.cc
	mp_generator.cc
	mp_g_delay_ms_condition.cc
	mp_g_common.cc
)

add_executable(mp_c
	mp_t_c.cc
)

target_link_libraries(mp_c mp)

target_link_libraries(mp mprobots ecp_mp)

install(TARGETS mp_c DESTINATION bin)
