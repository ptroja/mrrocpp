add_executable(ecp_build_tower
    ecp_t_build_tower.cc
)

target_link_libraries(ecp_build_tower ecp_r_irp6p_m ecp_generators ecp_subtasks ecp)

add_executable(mp_build_tower
	mp_t_build_tower.cc
)

target_link_libraries(mp_build_tower mp)

install(TARGETS ecp_build_tower mp_build_tower DESTINATION bin)