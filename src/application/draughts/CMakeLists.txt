if(ROBOT_IRP6_POSTUMENT)
add_executable(ecp_draughts
	ecp_g_vis_ib_eih_follower_irp6ot.cc
	ecp_mp_tr_draughtsAI.cc
	ecp_t_draughts.cc
	${ECPDIR}/irp6ot_m/generator/ecp_g_vis_ib_eih_planar_irp6ot.cc
	${ECPDIR}/common/task/ecp_st_go.cc
	${ECPDIR}/common/generator/ecp_g_sleep.cc
	${ECP_MPDIR}/sensor/ecp_mp_s_cvfradia.cc
)

target_link_libraries(ecp_draughts
	ecp_r_irp6ot_m
	ecp
)

install(TARGETS ecp_draughts DESTINATION bin)
endif(ROBOT_IRP6_POSTUMENT)
