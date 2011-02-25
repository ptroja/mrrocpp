add_executable(mp_rcsc
	mp_t_rcsc.cc
	ecp_mp_tr_rc_windows.cc
	CubeState.cc
	SingleManipulation.cc
)
add_executable(mp_fsautomat
	mp_t_fsautomat.cc
	ecp_mp_tr_rc_windows.cc
	CubeState.cc
	SingleManipulation.cc
	StateHeap.cc
	State.cc
	Condition.cc
	Transition.cc
)

add_executable(ecp_irp6ot_rcsc
	ecp_t_rcsc_irp6ot.cc
#	../../ecp/irp6_on_track/generator/ecp_g_vis_ib_eih_object_tracker_irp6ot.cc
    ../servovision/ib_eih_visual_servo.cc
    ../servovision/visual_servo.cc
    ../servovision/visual_servo_manager.cc
    ../servovision/single_visual_servo_manager.cc
    ../servovision/position_constraint.cc
    ../servovision/cubic_constraint.cc
    ../servovision/termination_condition.cc
    ../servovision/object_reached_termination_condition.cc
)
add_executable(ecp_irp6p_rcsc
	ecp_t_rcsc_irp6p.cc
)



add_executable(ecp_fsautomat
	ecp_t_fsautomat.cc
	../../generator/ecp/ecp_g_operator_reaction_condition.cc
)

target_link_libraries(mp_rcsc mp)
target_link_libraries(mp_fsautomat mp)

target_link_libraries(ecp_irp6ot_rcsc ecp_r_irp6ot_m
	ecp_generators ecp_subtasks)
target_link_libraries(ecp_irp6p_rcsc ecp_r_irp6p_m
	ecp_generators ecp_subtasks)
target_link_libraries(ecp_fsautomat ecp_r_irp6p_m ecp_r_irp6ot_m
	ecp_generators ecp_subtasks)

install(TARGETS
	mp_fsautomat ecp_fsautomat
	mp_rcsc
	ecp_irp6ot_rcsc ecp_irp6p_rcsc
	DESTINATION bin)
