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