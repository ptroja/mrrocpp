add_executable(mp_swarmitfix
	mp_t_swarmitfix.cc
)

target_link_libraries(mp_swarmitfix mp)

add_executable(ecp_spkm
	ecp_g_epos.cc
	ecp_t_spkm.cc
)

target_link_libraries(ecp_spkm
	ecp_r_spkm ecp
)

add_executable(ecp_smb
	ecp_g_epos.cc
	ecp_g_smb.cc
	ecp_t_smb.cc
)

target_link_libraries(ecp_smb
	ecp_r_smb ecp
)

add_executable(ecp_shead
	ecp_g_shead.cc
	ecp_t_shead.cc
)

target_link_libraries(ecp_shead
	ecp_r_shead ecp
)

install(TARGETS ecp_spkm ecp_smb ecp_shead mp_swarmitfix DESTINATION bin)
