# robienie zdjec obiektu z zadanych pozycji

add_executable(ecp_image_capture
    image_capture/capture_task.cc
)

target_link_libraries(ecp_image_capture
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)

####################################3
install(TARGETS ecp_image_capture DESTINATION bin)

add_subdirectory(objectfollower_ib)
add_subdirectory(objectfollower_pb_eih)
add_subdirectory(objectfollower_pb_sac)
add_subdirectory(visualservo_tester)