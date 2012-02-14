if(ROBOT_IRP6_POSTUMENT)
add_executable(ecp_dung
        ecp_g_dung.cc
        ecp_t_dung.cc
)

target_link_libraries(ecp_dung
        ecp_r_irp6p_m
        ecp
)

install(TARGETS ecp_dung DESTINATION bin)
endif(ROBOT_IRP6_POSTUMENT)
