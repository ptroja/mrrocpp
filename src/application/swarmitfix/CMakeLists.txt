include(FindXSD)
include(FindXerces)

if(XSD_FOUND AND XERCES_FOUND)

XSD_SCHEMA( PLAN_SRCS plan/plan.xsd --type-naming ucc --root-element plan )

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(plantest
	${PLAN_SRCS}
	plan/ParsePlan.cc
)

target_link_libraries(plantest ${XERCES_LIBRARIES})
endif(XSD_FOUND AND XERCES_FOUND)

add_executable(mp_swarmitfix
	mp_t_swarmitfix.cc
)

target_link_libraries(mp_swarmitfix mp)

add_executable(ecp_spkm
	ecp_g_spkm.cc
	ecp_t_spkm.cc
)

target_link_libraries(ecp_spkm ecp_r_spkm ecp 
)

#add_executable(ecp_smb
#	ecp_g_spkm.cc
#	ecp_g_smb.cc
#	ecp_t_smb.cc
#)

#target_link_libraries(ecp_smb
#	ecp_generators
#)

#add_executable(ecp_shead
#	ecp_g_shead.cc
#	ecp_t_shead.cc
#)

#target_link_libraries(ecp_shead
#	ecp_generators
#)

#install(TARGETS ecp_spkm ecp_smb ecp_shead mp_swarmitfix DESTINATION bin)
install(TARGETS ecp_spkm mp_swarmitfix DESTINATION bin)
