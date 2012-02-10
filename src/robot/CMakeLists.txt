# Recurse into subdirectories.


if(ROBOT_BIRD_HAND)
add_subdirectory (bird_hand)
endif(ROBOT_BIRD_HAND)


if(ROBOTS_012)
add_subdirectory (sarkofag)
add_subdirectory (conveyor)
add_subdirectory (festival)
add_subdirectory (irp6ot_tfg)
add_subdirectory (irp6p_tfg)
add_subdirectory (irp6_tfg)
add_subdirectory (irp6ot_m)
add_subdirectory (irp6p_m)
add_subdirectory (hi_moxa)
endif(ROBOTS_012)

add_library(mp_robots
	../base/mp/mp_robot.cc
)

target_link_library_if(ROBOTS_012 mp_robots mp_r_irp6ot_m)
target_link_library_if(ROBOTS_012 mp_robots mp_r_irp6ot_tfg)
target_link_library_if(ROBOTS_012 mp_robots mp_r_irp6p_m)
target_link_library_if(ROBOTS_012 mp_robots mp_r_irp6p_tfg)
target_link_library_if(ROBOTS_012 mp_robots mp_r_conv)
target_link_library_if(ROBOTS_012 mp_robots mp_r_sarkofag)
target_link_library_if(ROBOT_BIRD_HAND mp_robots mp_r_bird_hand)


add_library(ecp_robot
	../base/ecp/ecp_robot.cc
)

target_link_libraries(ecp_robot ecp_generators)

# enable conditional inclusion to ecp_robot
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_irp6ot_m)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_irp6ot_tfg)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_irp6p_m)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_irp6p_tfg)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_conv)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_sarkofag)
target_link_library_if(ROBOT_BIRD_HAND ecp_robot ecp_r_bird_hand)




install(TARGETS mp_robots ecp_robot DESTINATION lib)

