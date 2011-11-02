add_library(logger_client 
	logger_client.cc
	log_message.cc
)

target_link_libraries(logger_client ${Boost_SERIALIZATION_LIBRARY})

install(TARGETS logger_client DESTINATION lib)
