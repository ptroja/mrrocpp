add_executable(ecp_tran
	ecp_t_tran.cc
)

# this are dependency libraries
target_link_libraries(ecp_tran ecp_generators ecp_subtasks ecp_r_irp6ot_m)

install(TARGETS ecp_tran DESTINATION bin)
