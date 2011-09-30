add_executable(ecp_trapezoid_velocity
    ecp_g_trapezoid_velocity.cc
    ecp_t_trapezoid_velocity.cc
)

target_link_libraries(ecp_trapezoid_velocity
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)


################################
install(TARGETS ecp_trapezoid_velocity DESTINATION bin)
