add_library(agent DataBufferBase.cc AndDataCondition.cc OrDataCondition.cc AgentBase.cc Agent.cc)

target_link_libraries(agent ${Boost_SERIALIZATION_LIBRARY})


target_link_libraries(agent messip)


if(QNXNTO)
target_link_libraries(agent rpc)
endif(QNXNTO)

add_executable(reader reader.cc)

target_link_libraries(reader agent ${COMMON_LIBRARIES})

add_executable(writers writers.cc)
target_link_libraries(writers agent ${COMMON_LIBRARIES})

add_executable(DataConditionTest DataConditionTest.cc)
target_link_libraries(DataConditionTest agent ${COMMON_LIBRARIES})
