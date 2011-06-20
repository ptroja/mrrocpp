add_executable(ecp_tcim_bug
    ecp_t_tcim_bug.cc
    ecp_g_tcim_bug.cc
)

target_link_libraries(ecp_tcim_bug
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

####################################3
install(TARGETS ecp_tcim_bug DESTINATION bin)
