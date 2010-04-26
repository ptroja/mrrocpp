add_library(agent Agent.cc DataBuffer.cc AgentBase.cc)

if(USE_MESSIP)
target_link_libraries(agent messip)
endif(USE_MESSIP)

if(QNXNTO)
target_link_libraries(agent rpc)
endif(QNXNTO)

add_executable(reader reader.cc)
target_link_libraries(reader agent ${COMMON_LIBRARIES})

add_executable(writers writers.cc)
target_link_libraries(writers agent ${COMMON_LIBRARIES})
