add_subdirectory (generator)

add_library(mp
	mp_task_base.cc
	mp_task.cc
	mp_robot.cc
	mp_m.cc
)

target_link_libraries(mp mp_generator_base mp_robots ecp_mp)

install(TARGETS mp DESTINATION lib)
