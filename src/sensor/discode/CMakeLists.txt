add_library(discode_sensor
	discode_sensor.cc
)

target_link_libraries(discode_sensor ${Boost_SERIALIZATION_LIBRARY})

if(QNXNTO)
target_link_libraries (discode_sensor socket rpc)
endif(QNXNTO)

install(TARGETS discode_sensor DESTINATION lib)
