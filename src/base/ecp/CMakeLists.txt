MACRO(ADD_ECP_ROBOT NAME)
  add_library("ecp_r_${NAME}" "ecp_r_${NAME}.cc")
ENDMACRO(ADD_ECP_ROBOT)

MACRO(ADD_ECP_ROBOT_WITH_KINEMATICS NAME)
  ADD_ECP_ROBOT(${NAME})
  target_link_libraries(ecp_r_${NAME} kinematics${NAME})
ENDMACRO(ADD_ECP_ROBOT_WITH_KINEMATICS)

add_library(ecp
	ECP_main_error.cc
	ecp_m.cc
	ecp_task.cc
	ecp_sub_task.cc
	ecp_generator.cc
	ecp_taught_in_pose.cc
	ecp_g_transparent.cc
)

target_link_libraries(ecp ecp_subtasks ecp_generators ecp_robots ecp_mp)

install(TARGETS ecp DESTINATION bin) 