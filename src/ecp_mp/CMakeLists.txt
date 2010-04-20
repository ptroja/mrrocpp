add_library(ecp_mp
	ecp_mp_generator.cc
	sensor/ecp_mp_sensor.cc
	transmitter/transmitter.cc
	task/ecp_mp_task.cc
	ecp_mp_robot.cc
	Trajectory.cc
	smooth_trajectory_pose.cc
)

target_link_libraries(ecp_mp ${LIBXML2_LIBRARIES})
