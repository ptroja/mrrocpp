include(FindXSD)
include(FindXerces)

if(XSD_FOUND AND XERCES_FOUND)

XSD_SCHEMA( PLAN_SRCS plan.xsd --type-naming ucc --root-element plan --generate-serialization --generate-ostream --hxx-prologue-file ${CMAKE_CURRENT_SOURCE_DIR}/plan-prologue.hxx --generate-insertion boost::archive::text_oarchive --generate-extraction boost::archive::text_iarchive )

include_directories(${CMAKE_CURRENT_BINARY_DIR}) # for the "plan.hxx"

add_library(plan ${PLAN_SRCS})
target_link_libraries(plan ${XERCES_LIBRARIES})

add_executable(plantest ParsePlan.cc)

target_link_libraries(plantest plan ${Boost_THREAD_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} mrrocpp)

endif(XSD_FOUND AND XERCES_FOUND)

install(TARGETS plan DESTINATION lib)
install(FILES plan.xsd DESTINATION bin)
