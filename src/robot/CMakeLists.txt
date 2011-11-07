# Recurse into subdirectories.
add_subdirectory (sarkofag)
add_subdirectory (bird_hand)
add_subdirectory (conveyor)
add_subdirectory (shead)
add_subdirectory (smb)
add_subdirectory (spkm)
add_subdirectory (sbench)
add_subdirectory (canopen)
add_subdirectory (maxon)
add_subdirectory (festo)
add_subdirectory (festival)
add_subdirectory (player)
add_subdirectory (polycrank)
add_subdirectory (irp6ot_tfg)
add_subdirectory (irp6p_tfg)
add_subdirectory (irp6_tfg)
add_subdirectory (irp6ot_m)
add_subdirectory (irp6p_m)
add_subdirectory (hi_rydz)
add_subdirectory (hi_moxa)

add_library(mp_robots
	conveyor/mp_r_conveyor.cc
	irp6ot_m/mp_r_irp6ot_m.cc
	irp6p_m/mp_r_irp6p_m.cc
	polycrank/mp_r_polycrank.cc
	bird_hand/mp_r_bird_hand.cc
	spkm/mp_r_spkm.cc
	spkm/mp_r_spkm1.cc
	spkm/mp_r_spkm2.cc
	smb/mp_r_smb.cc
	smb/mp_r_smb1.cc
	smb/mp_r_smb2.cc
	shead/mp_r_shead.cc
	shead/mp_r_shead1.cc
	shead/mp_r_shead2.cc
	sbench/mp_r_sbench.cc
	irp6ot_tfg/mp_r_irp6ot_tfg.cc
	irp6p_tfg/mp_r_irp6p_tfg.cc
	sarkofag/mp_r_sarkofag.cc
)

add_library(ecp_robot
	../base/ecp/ecp_robot.cc
)

target_link_libraries(ecp_robot ecp_mp)

# enable conditional inclusion to ecp_robot
target_link_library_if(ROBOT_IRP6_ON_TRACK ecp_robot ecp_r_irp6ot_m)
target_link_library_if(ROBOT_IRP6_ON_TRACK ecp_robot ecp_r_irp6ot_tfg)
target_link_library_if(ROBOT_IRP6_POSTUMENT ecp_robot ecp_r_irp6p_m)
target_link_library_if(ROBOT_IRP6_POSTUMENT ecp_robot ecp_r_irp6p_tfg)
target_link_library_if(ROBOT_POLYCRANK ecp_robot ecp_r_polycrank)
target_link_library_if(ROBOT_CONVEYOR ecp_robot ecp_r_conv)
target_link_library_if(ROBOT_BIRD_HAND ecp_robot ecp_r_bird_hand)
target_link_library_if(ROBOT_SARKOFAG ecp_robot ecp_r_sarkofag)
target_link_library_if(ROBOT_SHEAD ecp_robot ecp_r_shead)
target_link_library_if(ROBOT_SMB ecp_robot ecp_r_smb)
target_link_library_if(ROBOT_SPKM ecp_robot ecp_r_spkm)
target_link_library_if(ROBOT_SBENCH ecp_robot ecp_r_sbench)

install(TARGETS mp_robots ecp_robot  DESTINATION lib)

