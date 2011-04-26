# object follower - IB - EIH
add_executable(ecp_vsdemo_objectfollower_ib_eih
    ecp_t_objectfollower_ib_eih.cc
)

target_link_libraries(ecp_vsdemo_objectfollower_ib_eih
	ecp_r_irp6p_m
	ecp_r_irp6ot_m
	visual_servoing
	ecp ${COMMON_LIBRARIES}
)

####################################
install(TARGETS ecp_vsdemo_objectfollower_ib_eih DESTINATION bin)
