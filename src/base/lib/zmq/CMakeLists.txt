if(${ZeroMQ_FOUND})

include_directories(${ZeroMQ_INCLUDE_DIRS})

add_library(zmqpp zmqpp.cc)
target_link_libraries(zmqpp ${ZeroMQ_LIBRARIES})

add_executable(zmq_names zmq_names.cc)
target_link_libraries (zmq_names ${ZeroMQ_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT} ${COMPATIBILITY_LIBRARIES} pthread)
	
add_executable(zmq_test zmq_test.cc)
target_link_libraries(zmq_test zmqpp ${Boost_THREAD_LIBRARY}) 

install(TARGETS zmq_names DESTINATION bin)

endif(${ZeroMQ_FOUND})
