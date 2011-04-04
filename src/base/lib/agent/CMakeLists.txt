add_library(agent DataBufferBase.cc AgentBase.cc Agent.cc)

target_link_libraries(agent ${Boost_SERIALIZATION_LIBRARY})

target_link_libraries(agent messip)



add_executable(reader reader.cc)

target_link_libraries(reader agent ${COMMON_LIBRARIES})

add_executable(writers writers.cc)
target_link_libraries(writers agent ${COMMON_LIBRARIES})
