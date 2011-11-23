add_executable(ecp_tran
	ecp_t_tran.cc
)

# this are dependency libraries
target_link_libraries(ecp_tran ecp_robot)

install(TARGETS ecp_tran DESTINATION bin)
