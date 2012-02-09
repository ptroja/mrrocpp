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

add_executable(ecp_irp6m_rcsc
	ecp_t_rcsc.cc
)


add_executable(ecp_fsautomat
	ecp_t_fsautomat.cc
)

target_link_libraries(mp_rcsc mp)
target_link_libraries(mp_fsautomat mp)

target_link_libraries(ecp_irp6m_rcsc
    ecp_robot
	)

target_link_libraries(ecp_fsautomat ecp_robot)

install(TARGETS
	mp_fsautomat ecp_fsautomat
	mp_rcsc
	ecp_irp6m_rcsc
	DESTINATION bin)
