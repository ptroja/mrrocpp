add_executable(ecp_tran
	../common/task/ecp_t_tran.cc
	../common/generator/ecp_g_transparent.cc
)

# enable conditional inclusion to this process
target_link_library_if(ROBOT_IRP6_ON_TRACK ecp_tran ecp_r_irp6ot_m)
target_link_library_if(ROBOT_IRP6_ON_TRACK ecp_tran ecp_r_irp6ot_tfg)
target_link_library_if(ROBOT_IRP6_POSTUMENT ecp_tran ecp_r_irp6p_m)
target_link_library_if(ROBOT_IRP6_POSTUMENT ecp_tran ecp_r_irp6p_tfg)
target_link_library_if(ROBOT_IRP6_MECHATRONICS ecp_tran ecp_r_irp6m)
target_link_library_if(ROBOT_POLYCRANK ecp_tran ecp_r_polycrank)
target_link_library_if(ROBOT_CONVEYOR ecp_tran ecp_r_conv)
target_link_library_if(ROBOT_SPEAKER ecp_tran ecp_r_speaker)

# this are dependency libraries
target_link_libraries(ecp_tran ecp)

install(TARGETS ecp_tran DESTINATION bin)
