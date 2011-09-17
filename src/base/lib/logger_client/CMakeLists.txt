add_library(logger_client 
	logger_client.cc
)

target_link_libraries(logger_client ${Boost_SERIALIZATION_LIBRARY})

install(TARGETS logger_client DESTINATION lib)
