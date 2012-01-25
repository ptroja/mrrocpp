add_executable(ecp_android_teach
	sensor/EcpMpAndroid.cc
	AndroidTeach.cc
	AndroidState.cc
	generator/EcpGAndroidJoint.cc
	generator/EcpGAndroid.cc
	generator/EcpSmoothGAndroid.cc
	
)

target_link_libraries(ecp_android_teach
	ecp_robot
)

install(TARGETS ecp_android_teach DESTINATION bin)
