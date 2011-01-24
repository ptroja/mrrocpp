# object follower - IB - EIH
add_executable(ecp_vsdemo_vs_tutorial
    ecp_t_vs_tutorial.cc
)

target_link_libraries(ecp_vsdemo_vs_tutorial
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	visual_servoing
	ecp ${COMMON_LIBRARIES}
)

####################################
install(TARGETS ecp_vsdemo_vs_tutorial DESTINATION bin)
