include(FindXSD)
include(FindXerces)

if(XSD_FOUND AND XERCES_FOUND)

XSD_SCHEMA( PLAN_SRCS plan.xsd --type-naming ucc --root-element plan --generate-serialization --generate-ostream --hxx-prologue-file ${CMAKE_CURRENT_SOURCE_DIR}/plan-prologue.hxx --generate-insertion boost::archive::text_oarchive --generate-extraction boost::archive::text_iarchive )

include_directories(${CMAKE_CURRENT_BINARY_DIR}) # for the "plan.hxx"

add_library(plan ${PLAN_SRCS} plan_iface.cc)
target_link_libraries(plan ${XERCES_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})

add_executable(plantest ParsePlan.cc)

target_link_libraries(plantest plan mrrocpp)

endif(XSD_FOUND AND XERCES_FOUND)

install(TARGETS plan DESTINATION lib)
install(FILES plan.xsd DESTINATION bin)
