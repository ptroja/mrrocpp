if(${ZeroMQ_FOUND})

include_directories(${ZeroMQ_INCLUDE_DIRS})

add_library(zmqps context.cc registry.cc publisher.cc subscriber.cc)
target_link_libraries(zmqps ${ZeroMQ_LIBRARIES}
    ${Boost_THREAD_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_executable(zmq_names zmq_names.cc)
target_link_libraries (zmq_names zmqps)
	
add_executable(test_subscribe test_subscribe.cc)
target_link_libraries(test_subscribe zmqps)

add_executable(test_publish test_publish.cc)
target_link_libraries(test_publish zmqps) 

install(TARGETS zmq_names DESTINATION bin)
install(TARGETS zmqps DESTINATION lib)

endif(${ZeroMQ_FOUND})
