# object follower - PB - SAC
add_executable(ecp_trapezoid
    ecp_t_trapezoid_task.cc
    ecp_g_trapezoid_generator.cc
)

target_link_libraries(ecp_trapezoid
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)

####################################3
install(TARGETS ecp_trapezoid DESTINATION bin)
