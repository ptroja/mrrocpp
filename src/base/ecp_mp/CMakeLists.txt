add_library(ecp_mp
	ecp_mp_generator.cc
	transmitter.cc
	ecp_mp_task.cc
	ecp_mp_robot.cc
	Trajectory.cc
)

target_link_libraries(ecp_mp ecp_mp_generators ${LIBXML2_LIBRARIES} ${COMMON_LIBRARIES} agent)

install(TARGETS ecp_mp DESTINATION lib)
