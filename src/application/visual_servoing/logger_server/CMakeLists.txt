add_executable(logger_server
    logger_server_main.cc
    logger_server.cc
    client_connection.cc
)

# link with discode_sensor
target_link_libraries(logger_server
	${COMMON_LIBRARIES}
	logger_client
)

install(TARGETS logger_server DESTINATION bin)
