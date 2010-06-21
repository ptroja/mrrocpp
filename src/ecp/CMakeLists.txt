MACRO(ADD_ECP_ROBOT NAME)
  add_library("ecp_r_${NAME}" "ecp_r_${NAME}.cc")
ENDMACRO(ADD_ECP_ROBOT)

MACRO(ADD_ECP_ROBOT_WITH_KINEMATICS NAME)
  ADD_ECP_ROBOT(${NAME})
  target_link_libraries(ecp_r_${NAME} kinematics${NAME})
ENDMACRO(ADD_ECP_ROBOT_WITH_KINEMATICS)


add_library(ecp
	ECP_main_error.cc
	ecp_robot.cc
	ecp_m.cc
	task/ecp_task.cc
	task/ecp_st_bias_edp_force.cc
	task/ecp_st_tff_nose_run.cc
	generator/ecp_generator.cc
	generator/ecp_g_teach_in.cc
	generator/ecp_g_delta.cc
	generator/ecp_g_operator_reaction_condition.cc
	generator/ecp_g_jarosz.cc
	generator/ecp_g_visual_servo_manager.cc
	generator/ecp_g_visual_servo.cc
	ecp_taught_in_pose.cc
	generator/ecp_g_smooth.cc
	generator/ecp_g_sleep.cc
	generator/ecp_g_force.cc
	generator/ecp_g_transparent.cc
	generator/ecp_g_get_position.cc
	generator/ecp_g_multiple_position.cc
	generator/ecp_g_newsmooth.cc
	generator/velocity_profile_calculator/bang_bang_profile.cc
	generator/velocity_profile_calculator/constant_velocity_profile.cc
	generator/velocity_profile_calculator/velocity_profile.cc
	generator/trajectory_interpolator/trajectory_interpolator.cc
	generator/trajectory_interpolator/constant_velocity_interpolator.cc
	
)

target_link_libraries(ecp ecp_mp)


add_executable(ecp_tran
	task/ecp_t_tran.cc
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

