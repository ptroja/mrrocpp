

add_library(agent BufferBase.cc InputBufferBase.cc AgentBase.cc Agent.cc RemoteAgent.cc)

# Serialization is essential for this library
target_link_libraries(agent ${Boost_SERIALIZATION_LIBRARY})

# Link the following transports 
target_link_libraries(agent messip)

install(TARGETS agent DESTINATION lib)

if(QNXNTO)
target_link_libraries(agent rpc)
endif(QNXNTO)


#
# Test programs
#
#add_executable(reader reader.cc)
#target_link_libraries(reader agent ${COMMON_LIBRARIES})
#
#add_executable(writers writers.cc)
#target_link_libraries(writers agent ${COMMON_LIBRARIES})
