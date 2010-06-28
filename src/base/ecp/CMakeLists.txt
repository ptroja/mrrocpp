MACRO(ADD_ECP_ROBOT NAME)
  add_library("ecp_r_${NAME}" "ecp_r_${NAME}.cc")
ENDMACRO(ADD_ECP_ROBOT)

MACRO(ADD_ECP_ROBOT_WITH_KINEMATICS NAME)
  ADD_ECP_ROBOT(${NAME})
  target_link_libraries(ecp_r_${NAME} kinematics${NAME})
ENDMACRO(ADD_ECP_ROBOT_WITH_KINEMATICS)


add_library(ecp
	ecp_robot.cc
	ecp_m.cc
	ecp_task.cc
	ecp_generator.cc
	ecp_taught_in_pose.cc
	ecp_g_transparent.cc
)

target_link_libraries(ecp ecp_generators ecp_subtasks ecp_mp)


add_executable(ecp_tran
	ecp_t_tran.cc
)

# enable conditional inclusion to this process
target_link_library_if(ROBOT_IRP6_ON_TRACK ecp_tran ecp_r_irp6ot_m)
target_link_library_if(ROBOT_IRP6_ON_TRACK ecp_tran ecp_r_irp6ot_tfg)
target_link_library_if(ROBOT_IRP6_POSTUMENT ecp_tran ecp_r_irp6p_m)
target_link_library_if(ROBOT_IRP6_POSTUMENT ecp_tran ecp_r_irp6p_tfg)
target_link_library_if(ROBOT_IRP6_MECHATRONICS ecp_tran ecp_r_irp6m)
target_link_library_if(ROBOT_POLYCRANK ecp_tran ecp_r_polycrank)
target_link_library_if(ROBOT_CONVEYOR ecp_tran ecp_r_conv)
target_link_library_if(ROBOT_SPEAKER ecp_tran ecp_r_speaker)

# this are dependency libraries
target_link_libraries(ecp_tran ecp)

install(TARGETS ecp_tran DESTINATION bin)

