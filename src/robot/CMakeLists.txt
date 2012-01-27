# Recurse into subdirectories.


if(ROBOT_BIRD_HAND)
add_subdirectory (bird_hand)
endif(ROBOT_BIRD_HAND)

if(ROBOTS_SWARMITFIX)
    # Hardware interface.
    add_subdirectory (canopen)
    add_subdirectory (maxon)
    add_subdirectory (festo)
    
    # Robots for the agents.
    add_subdirectory (shead)
    add_subdirectory (smb)
    add_subdirectory (spkm)
    
    if(NOT UBUNTU32BIT)
        add_subdirectory (sbench)
    endif(NOT UBUNTU32BIT)
endif(ROBOTS_SWARMITFIX)

if(ROBOTS_012)
if(NOT UBUNTU32BIT)
add_subdirectory (sarkofag)
add_subdirectory (conveyor)
add_subdirectory (festival)
add_subdirectory (irp6ot_tfg)
add_subdirectory (irp6p_tfg)
add_subdirectory (irp6_tfg)
add_subdirectory (irp6ot_m)
add_subdirectory (irp6p_m)
add_subdirectory (hi_moxa)
    endif(NOT UBUNTU32BIT)
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
target_link_library_if(ROBOTS_SWARMITFIX mp_robots mp_r_spkm )
target_link_library_if(ROBOTS_SWARMITFIX mp_robots mp_r_shead)
target_link_library_if(ROBOTS_SWARMITFIX mp_robots mp_r_smb)
target_link_library_if(ROBOTS_SWARMITFIX mp_robots mp_r_sbench)

add_library(ecp_robot
	../base/ecp/ecp_robot.cc
)


# enable conditional inclusion to ecp_robot
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_irp6ot_m)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_irp6ot_tfg)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_irp6p_m)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_irp6p_tfg)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_conv)
target_link_library_if(ROBOTS_012 ecp_robot ecp_r_sarkofag)
target_link_library_if(ROBOT_BIRD_HAND ecp_robot ecp_r_bird_hand)
target_link_library_if(ROBOTS_SWARMITFIX ecp_robot ecp_r_spkm)
target_link_library_if(ROBOTS_SWARMITFIX ecp_robot ecp_r_shead )
target_link_library_if(ROBOTS_SWARMITFIX ecp_robot  ecp_r_smb )
target_link_library_if(ROBOTS_SWARMITFIX ecp_robot  ecp_r_sbench)

target_link_libraries(ecp_robot ecp)

install(TARGETS mp_robots ecp_robot DESTINATION lib)

