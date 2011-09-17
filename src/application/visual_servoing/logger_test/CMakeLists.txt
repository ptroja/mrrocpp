FIND_PACKAGE(Boost REQUIRED program_options)

add_executable(logger_test
    logger_test.cc
)

# link with discode_sensor
target_link_libraries(logger_test
	${COMMON_LIBRARIES}
	${Boost_PROGRAM_OPTIONS_LIBRARY}
	logger_client
)

install(TARGETS logger_test DESTINATION bin)
