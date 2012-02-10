add_executable(ecp_spring_contact
	ecp_g_spring_contact.cc
	ecp_t_spring_contact.cc
)

target_link_libraries(ecp_spring_contact ecp_robot)

add_executable(mp_spring_contact
	mp_t_spring_contact.cc
)

target_link_libraries(mp_spring_contact mp)

install(TARGETS ecp_spring_contact mp_spring_contact DESTINATION bin)
