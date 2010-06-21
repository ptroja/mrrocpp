add_library(ecp_mp
	ecp_mp_generator.cc
	transmitter.cc
	ecp_mp_task.cc
	ecp_mp_robot.cc
	Trajectory.cc
	smooth_trajectory_pose.cc
	../lib/trajectory_pose/bang_bang_trajectory_pose.cc
	../lib/trajectory_pose/trajectory_pose.cc
	../lib/trajectory_pose/constant_velocity_trajectory_pose.cc
)

target_link_libraries(ecp_mp ${LIBXML2_LIBRARIES} ${COMMON_LIBRARIES})
