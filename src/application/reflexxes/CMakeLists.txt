include_directories(../../contrib/Reflexxes/include)

add_executable(ecp_reflexxes
	ecp_t_reflexxes.cc
	ecp_g_reflexxes.cc
)

target_link_libraries(ecp_reflexxes Reflexxes ecp_robot)

install(TARGETS ecp_reflexxes DESTINATION bin)
