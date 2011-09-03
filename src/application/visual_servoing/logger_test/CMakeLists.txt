add_executable(logger_test
    logger_test.cc
)

# link with discode_sensor
target_link_libraries(logger_test
	${COMMON_LIBRARIES}
)

install(TARGETS logger_test DESTINATION bin)
