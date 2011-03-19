

add_executable(mp_c
	mp_t_c.cc
)

target_link_libraries(mp_c mp)

install(TARGETS mp_c DESTINATION bin)
