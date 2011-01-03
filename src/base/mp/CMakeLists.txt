add_subdirectory (generator)

add_library(mp
    MP_main_error.cc
	mp_task.cc
	mp_robot.cc
	mp_m.cc
)

target_link_libraries(mp mp_generator_base mprobots ecp_mp)


