add_library(mp
	task/mp_task.cc
	robot/mp_robot.cc
	mp_m.cc
	generator/mp_generator.cc
	generator/mp_g_delay_ms_condition.cc
	generator/mp_g_common.cc
)

add_library(mprobots
	robot/mp_r_conveyor.cc
	robot/mp_r_irp6_on_track.cc
	robot/mp_r_irp6_postument.cc
	robot/mp_r_speaker.cc
	robot/mp_r_irp6_mechatronika.cc
	robot/mp_r_polycrank.cc
	robot/mp_r_spkm.cc
	robot/mp_r_smb.cc
	robot/mp_r_shead.cc
	robot/mp_r_irp6ot_tfg.cc
	robot/mp_r_irp6p_tfg.cc
	robot/mp_r_manip_and_conv.cc
)

add_executable(mp_c
	task/mp_t_c.cc
)

target_link_libraries(mp_c mp)

target_link_libraries(mp mprobots ecp_mp)

install(TARGETS mp_c DESTINATION bin)
