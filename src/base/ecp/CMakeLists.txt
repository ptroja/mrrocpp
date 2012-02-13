MACRO(ADD_ECP_ROBOT NAME)
  add_library("ecp_r_${NAME}" "ecp_r_${NAME}.cc")
  install(TARGETS "ecp_r_${NAME}" DESTINATION lib)
ENDMACRO(ADD_ECP_ROBOT)

MACRO(ADD_ECP_ROBOT_WITH_KINEMATICS NAME)
  ADD_ECP_ROBOT(${NAME})
  target_link_libraries(ecp_r_${NAME} kinematics${NAME})
ENDMACRO(ADD_ECP_ROBOT_WITH_KINEMATICS)


add_library(ecp
	ecp_m.cc
	ecp_task.cc
	ecp_robot.cc
)

target_link_libraries(ecp ecp_mp)
	
install(TARGETS ecp DESTINATION lib)
