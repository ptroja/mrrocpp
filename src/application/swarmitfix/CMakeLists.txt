include(FindXSD)
include(FindXerces)

if(XSD_FOUND AND XERCES_FOUND)

XSD_SCHEMA( PLAN_SRCS plan/plan.xsd --type-naming ucc --root-element plan --generate-serialization --generate-ostream --hxx-prologue-file ${CMAKE_CURRENT_SOURCE_DIR}/plan-prologue.hxx --generate-insertion boost::archive::text_oarchive --generate-extraction boost::archive::text_iarchive )

include_directories(${CMAKE_CURRENT_BINARY_DIR}) # for the "plan.hxx"

add_library(plan ${PLAN_SRCS})
target_link_libraries(plan ${XERCES_LIBRARIES})

add_executable(plantest
	ParsePlan.cc planner.cc
)

target_link_libraries(plantest plan ${Boost_THREAD_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} mrrocpp)

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
install(TARGETS plan DESTINATION lib)
install(FILES plan/plan.xsd DESTINATION bin)
