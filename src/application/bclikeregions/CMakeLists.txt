add_executable(ecp_bclikeregions
    bclikeregions_task.cc
    bclikeregions_gen.cc
    bclike_smooth.cc
    ../servovision/position_constraint.cc    
    ../servovision/cubic_constraint.cc
)

add_executable(ecp_bclike_mp
	bclike_mp.cc
)

add_executable(ecp_bcl_t_switcher
	bcl_t_switcher.cc
    bclike_smooth.cc
    ecp_st_smooth_move.cc
)

target_link_libraries(ecp_bclikeregions
    ecp_r_irp6ot_m
    ecp ${COMMON_LIBRARIES}
)

target_link_libraries(ecp_bcl_t_switcher
    
    ecp_r_irp6ot_m
    ecp ${COMMON_LIBRARIES}
)

target_link_libraries(ecp_bclike_mp mp)

#########################################
install(TARGETS ecp_bclikeregions ecp_bclike_mp ecp_bcl_t_switcher DESTINATION bin)