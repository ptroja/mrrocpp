add_executable(ecp_trajectoryline
    ecp_g_trajectoryline.cc
    ecp_t_trajectoryline.cc
)

target_link_libraries(ecp_trajectoryline
	ecp_r_irp6ot_m
	ecp
	${COMMON_LIBRARIES}
)

################################
install(TARGETS ecp_trajectoryline DESTINATION bin)
