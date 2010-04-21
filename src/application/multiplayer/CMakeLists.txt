add_executable(mp_multiplayer
	mp_t_multiplayer.cc
)

target_link_libraries(mp_multiplayer
	mpmain mprobots ecp_mp
	${COMMON_LIBRARIES}
)

add_executable(ecp_irp6ot_multiplayer
	ecp_t_multiplayer_irp6ot.cc
	${ECPDIR}/common/task/ecp_st_go.cc
)

target_link_libraries(ecp_irp6ot_multiplayer
	ecp_r_irp6ot ecp
	${COMMON_LIBRARIES}
)

install(TARGETS mp_multiplayer ecp_irp6ot_multiplayer DESTINATION bin)
