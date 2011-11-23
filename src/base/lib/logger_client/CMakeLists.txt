add_library(logger_client 
	logger_client.cc
	log_message.cc
)

target_link_libraries(logger_client ${Boost_THREAD_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} mrrocpp)

install(TARGETS logger_client DESTINATION lib)
