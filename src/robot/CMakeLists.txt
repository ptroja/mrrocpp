# Recurse into subdirectories.
add_subdirectory (sarkofag)
add_subdirectory (bird_hand)
add_subdirectory (conveyor)
add_subdirectory (shead)
add_subdirectory (smb)
add_subdirectory (speaker)
add_subdirectory (spkm)
add_subdirectory (epos)
add_subdirectory (festival)
add_subdirectory (player)
add_subdirectory (polycrank)
add_subdirectory (irp6m)
add_subdirectory (irp6ot_tfg)
add_subdirectory (irp6p_tfg)
add_subdirectory (irp6_tfg)
add_subdirectory (irp6ot_m)
add_subdirectory (irp6p_m)
add_subdirectory (hi_rydz)
add_subdirectory (hi_moxa)


add_library(mprobots
	conveyor/mp_r_conveyor.cc
	irp6ot_m/mp_r_irp6ot_m.cc
	irp6p_m/mp_r_irp6p_m.cc
	speaker/mp_r_speaker.cc
	irp6m/mp_r_irp6m.cc
	polycrank/mp_r_polycrank.cc
	bird_hand/mp_r_bird_hand.cc
	spkm/mp_r_spkm.cc
	smb/mp_r_smb.cc
	shead/mp_r_shead.cc
	irp6ot_tfg/mp_r_irp6ot_tfg.cc
	irp6p_tfg/mp_r_irp6p_tfg.cc
	sarkofag/mp_r_sarkofag.cc
)

add_library(ecp_robots
	../base/ecp/ecp_robot.cc
)

# enable conditional inclusion to ecp_robots
target_link_library_if(ROBOT_IRP6_ON_TRACK ecp_robots ecp_r_irp6ot_m)
target_link_library_if(ROBOT_IRP6_ON_TRACK ecp_robots ecp_r_irp6ot_tfg)
target_link_library_if(ROBOT_IRP6_POSTUMENT ecp_robots ecp_r_irp6p_m)
target_link_library_if(ROBOT_IRP6_POSTUMENT ecp_robots ecp_r_irp6p_tfg)
target_link_library_if(ROBOT_IRP6_MECHATRONICS ecp_robots ecp_r_irp6m)
target_link_library_if(ROBOT_POLYCRANK ecp_robots ecp_r_polycrank)
target_link_library_if(ROBOT_CONVEYOR ecp_robots ecp_r_conv)
target_link_library_if(ROBOT_SPEAKER ecp_robots ecp_r_speaker)
target_link_library_if(ROBOT_BIRD_HAND ecp_robots ecp_r_bird_hand)
target_link_library_if(ROBOT_SARKOFAG ecp_robots ecp_r_sarkofag)
target_link_library_if(ROBOT_SHEAD ecp_robots ecp_r_shead)
target_link_library_if(ROBOT_SMB ecp_robots ecp_r_smb)
target_link_library_if(ROBOT_SPKM ecp_robots ecp_r_spkm)