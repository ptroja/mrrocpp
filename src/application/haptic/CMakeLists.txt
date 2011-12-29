
add_executable(mp_haptic
	mp_t_haptic.cc
	mp_g_haptic.cc
)

target_link_libraries(mp_haptic mp)

install(TARGETS mp_haptic DESTINATION bin)
