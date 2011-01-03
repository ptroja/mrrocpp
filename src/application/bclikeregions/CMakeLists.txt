add_executable(ecp_bclike_mp_test
	bclike_mp_test.cc
	ecp_mp_message.cc
)

add_executable(ecp_bclike_mp_i
	bclike_mp_i.cc
	ecp_mp_message.cc
)

add_executable(ecp_bclike_mp_ui
	bclike_mp_ui.cc
	ecp_mp_message.cc
)

add_executable(ecp_bcl_t_switcher
	bcl_t_switcher.cc
    bclike_gen.cc
    ecp_st_scan_move.cc
    ecp_st_position_move.cc
    ecp_mp_message.cc
)


target_link_libraries(ecp_bcl_t_switcher
    ecp_r_irp6ot_m
    ecp_r_irp6p_m
    ecp ${COMMON_LIBRARIES}
)


target_link_libraries(ecp_bclike_mp_test mp)
target_link_libraries(ecp_bclike_mp_ui mp)
target_link_libraries(ecp_bclike_mp_i mp)
#########################################
install(TARGETS ecp_bclike_mp_test ecp_bclike_mp_i ecp_bclike_mp_ui ecp_bcl_t_switcher DESTINATION bin)