if(XSD_FOUND AND XERCES_FOUND)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/../swarmitfix_plan) # for the "plan.hxx"

add_executable(mp_swarmitfix
	mp_t_swarmitfix.cc mp_t_swarmitfix_logic.cc mp_t_swarmtest.cc planner.cc WorkersStatus.cc
)

target_link_libraries(mp_swarmitfix plan ${XERCES_LIBRARIES} mp ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

install(TARGETS mp_swarmitfix DESTINATION bin)

endif(XSD_FOUND AND XERCES_FOUND)

add_executable(ecp_spkm
	ecp_g_spkm.cc
	ecp_t_spkm.cc
)

target_link_libraries(ecp_spkm ecp_r_spkm ecp)

add_executable(ecp_smb
	ecp_g_smb.cc
	ecp_t_smb.cc
)

target_link_libraries(ecp_smb ecp_r_smb ecp)

add_executable(ecp_sbench
	ecp_g_sbench.cc
	ecp_t_sbench.cc
)

target_link_libraries(ecp_sbench ecp_r_sbench ecp)

add_executable(ecp_shead
	ecp_g_shead.cc
	ecp_t_shead.cc
)

target_link_libraries(ecp_shead ecp_r_shead ecp)

install(TARGETS ecp_spkm ecp_smb ecp_sbench ecp_shead DESTINATION bin)
