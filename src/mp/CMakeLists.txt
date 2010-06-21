add_library(mp
	task/mp_task.cc
	robot/mp_robot.cc
	mp_m.cc
	generator/mp_generator.cc
	generator/mp_g_delay_ms_condition.cc
	generator/mp_g_common.cc
)

add_library(mprobots
	../robot/conveyor/mp_r_conveyor.cc
	robot/mp_r_irp6ot_m.cc
	robot/mp_r_irp6p_m.cc
	../robot/speaker/mp_r_speaker.cc
	../robot/irp6_mechatronika/mp_r_irp6_mechatronika.cc
	../robot/polycrank/mp_r_polycrank.cc
	../robot/bird_hand/mp_r_bird_hand.cc
	../robot/spkm/mp_r_spkm.cc
	../robot/smb/mp_r_smb.cc
	../robot/shead/mp_r_shead.cc
	../robot/irp6ot_tfg/mp_r_irp6ot_tfg.cc
	../robot/irp6p_tfg/mp_r_irp6p_tfg.cc
	robot/mp_r_motor_driven.cc
)

add_executable(mp_c
	task/mp_t_c.cc
)

target_link_libraries(mp_c mp)

target_link_libraries(mp mprobots ecp_mp)

install(TARGETS mp_c DESTINATION bin)
