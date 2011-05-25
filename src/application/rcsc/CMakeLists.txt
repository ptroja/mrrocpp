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
)
add_executable(ecp_irp6p_rcsc
	ecp_t_rcsc_irp6p.cc
)



add_executable(ecp_fsautomat
	ecp_t_fsautomat.cc
)

target_link_libraries(mp_rcsc mp)
target_link_libraries(mp_fsautomat mp)

target_link_libraries(ecp_irp6ot_rcsc
    ecp_r_irp6ot_m
	ecp_generators 
	ecp_subtasks
	visual_servoing
	)
target_link_libraries(ecp_irp6p_rcsc ecp_r_irp6p_m
	ecp_generators ecp_subtasks)
target_link_libraries(ecp_fsautomat ecp_r_irp6p_m ecp_r_irp6ot_m
	ecp_generators ecp_subtasks)

install(TARGETS
	mp_fsautomat ecp_fsautomat
	mp_rcsc
	ecp_irp6ot_rcsc ecp_irp6p_rcsc
	DESTINATION bin)
