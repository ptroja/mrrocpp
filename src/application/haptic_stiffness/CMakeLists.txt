
add_executable(mp_haptic_stiffness
	mp_t_haptic_stiffness.cc
	mp_g_haptic_stiffness.cc
)

target_link_libraries(mp_haptic_stiffness mp)

install(TARGETS mp_haptic_stiffness DESTINATION bin)
