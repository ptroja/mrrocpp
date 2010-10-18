/* ../MAM_wnd_manual_moves_automatic_measure.cc */
int MAM_btn_start_measures(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int MAM_btn_stop_measures(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int MAM_btn_clear_measures(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int MAM_btn_save_measures(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int MAM_btn_calibrate(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int
MAM_btn_exit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int MAM_tmr_connect(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int MAM_refresh_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../abmain.cc */

/* ../fun.cc */
int set_ui_busy_state_notification(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int set_ui_ready_state_notification(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion);
int close_process_control_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_teaching_window_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_file_selection_window_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_process_control_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_process_control_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_task_config_window_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_task_config_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int yes_no_callback(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int input_integer_callback(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int input_double_callback(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int choose_option_callback(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_file_selection_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_teaching_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_task_config_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_teaching_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int teaching_window_end_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int file_selection_window_send_location(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int file_selection_window_post_realize(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_base_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int quit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int process_control_window_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int block_all_ecp_trigger_widgets(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int unblock_all_ecp_trigger_widgets(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int task_param_actualization(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int task_window_param_actualization(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int manage_configuration_file(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int start_file_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int clear_console(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int unload_all(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int slay_all(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int activate_menu_from_widget(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int activate_file_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int activate_robot_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int activate_all_robots_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int activate_task_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int activate_special_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int activate_help_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_yes_no_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_input_integer_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_input_double_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_choose_option_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_all_robots_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int teaching_window_send_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int all_robots_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_all_robots_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_all_robots_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int MPup(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int MPup_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int MPslay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_start_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int pulse_stop_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int pulse_pause_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int pulse_resume_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int pulse_trigger_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int signal_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int pulse_reader_all_robots_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int pulse_reader_all_robots_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int pulse_reader_all_robots_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_ecp_all_robots(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_conveyor.cc */
int close_wind_conveyor_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_conveyor_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_conveyor_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wind_conveyor_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wind_conveyor_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_conveyor_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wind_conveyor_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int conveyor_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wind_conveyor_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_conveyor_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_conveyor_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_conveyor_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int conv_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_conveyor_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_conveyor_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_conveyor_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_conv_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_conv_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_conv_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_ecp_conveyor(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_irp6_mechatronika.cc */
int init_wnd_irp6m_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6m_xyz_euler_zyz_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6m_xyz_zyz_ts_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_mechatronika_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_mechatronika_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int EDP_irp6_mechatronika_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6m_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6m_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6m_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6m_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6m_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6m_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6m_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int pulse_reader_irp6m_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_irp6m_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_irp6m_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_ecp_irp6_mechatronika(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6m_motors_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_inc_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6m_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6m_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6m_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6m_joints_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_int_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6m_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6m_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6m_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6m_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_kinematic_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6m_ser_alg_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6m_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6m_xyz_aa_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_xyz_angle_axis_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6m_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6m_xyz_aa_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_xyz_angle_axis_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6m_xyz_aa_ts_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6m_xyz_zyz_copy_cur_to_des(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_xyz_euler_zyz_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6m_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6m_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6m_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_xyz_euler_zyz_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6m_inc_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6m_int_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6m_kinematic_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6m_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6m_xyz_angle_axis_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6m_xyz_aa_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6m_xyz_euler_zyz_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_mechatronika_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_mechatronika_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_mechatronika_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_mechatronika_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_mechatronika_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_mechatronika_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_mechatronika_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_mechatronika_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6m_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int import_wnd_irp6m_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6m_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_irp6_on_track.cc */
int close_wnd_irp6_on_track_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_on_track_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_on_track_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_on_track_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_on_track_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6ot_xyz_angle_axis_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6ot_kinematic_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6ot_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_on_track_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_on_track_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_on_track_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6ot_xyz_euler_zyz_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_on_track_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_on_track_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_on_track_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_on_track_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_on_track_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_on_track_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_on_track_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6_on_track_inc_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6_on_track_int_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6ot_xyz_euler_zyz_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6ot_xyz_angle_axis_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6ot_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6_on_track_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6_on_track_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6_on_track_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6_on_track_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6_on_track_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6_on_track_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_on_track_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_on_track_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6ot_motors_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6ot_joints_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_execute_motor_motion(void);
int irp6ot_execute_joint_motion(void);
int irp6ot_inc_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_on_track_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_int_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_on_track_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6ot_xyz_zyz_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_xyz_euler_zyz_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_on_track_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6ot_xyz_aa_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_xyz_angle_axis_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_on_track_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_on_track_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_on_track_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6ot_xyz_aa_ts_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_xyz_angle_axis_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_on_track_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6ot_xyz_zyz_ts_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_xyz_euler_zyz_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_on_track_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_kinematic_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_on_track_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6ot_seralg_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_on_track_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_on_track_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_on_track_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_irp6ot_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_irp6ot_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_irp6ot_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_ecp_irp6_on_track(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int start_wnd_irp6_on_track_xyz_aa_relative(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6ot_xyz_aa_relative_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_on_track_xyz_aa_relative(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_xyz_aa_relative_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6_on_track_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6_on_track_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_irp6_postument.cc */
int close_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_postument_xyz_aa_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_xyz_angle_axis_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_postument_xyz_angle_axis_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_kinematic_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_postument_xyz_eul_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_postument_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_postument_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_xyz_euler_zyz_ts_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_postument_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_postument_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_postument_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_inc_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_int_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_xyz_euler_zyz_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_xyz_angle_axis_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_postument_inc(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6p_motors_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_execute_motor_motion(void);
int irp6p_execute_joint_motion(void);
int irp6p_inc_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_postument_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6p_joints_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_int_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_postument_xyz_euler_zyz(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6p_xyz_zyz_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_xyz_euler_zyz_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6p_xyz_aa_copy_current_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_xyz_angle_axis_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_postument_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_postument_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_postument_xyz_aa_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6p_xyz_aa_ts_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_xyz_angle_axis_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_postument_xyz_euler_zyz_ts(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6p_xyz_zyz_ts_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_xyz_euler_zyz_set_tool(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_postument_kinematic(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_kinematic_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6_postument_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wnd_irp6p_ser_alg_copy_cur_to_desired(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_postument_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_postument_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6_postument_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_irp6p_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_irp6p_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_irp6p_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_ecp_irp6_postument(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int start_wnd_irp6_postument_xyz_aa_relative(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_xyz_aa_relative_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6_postument_xyz_aa_relative(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_xyz_aa_relative_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int import_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int export_wnd_irp6_postument_xyz_angle_axis(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_irp6ot_tfg.cc */
int close_wind_irp6ot_tfg_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6ot_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6ot_tfg_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6ot_tfg_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6ot_tfg_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int EDP_irp6ot_tfg_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6ot_tfg_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wind_irp6ot_tfg_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_tfg_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_tfg_execute_motor_motion(void);
int irp6ot_tfg_execute_joint_motion(void);
int start_wnd_irp6ot_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6ot_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6ot_tfg_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wind_irp6ot_tfg_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wind_irp6ot_tfg_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wind_irp6ot_tfg_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6ot_tfg_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_irp6p_tfg.cc */
int close_wind_irp6p_tfg_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_irp6p_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6p_tfg_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6p_tfg_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6p_tfg_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int EDP_irp6p_tfg_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_irp6p_tfg_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wind_irp6p_tfg_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_tfg_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_tfg_execute_motor_motion(void);
int irp6p_tfg_execute_joint_motion(void);
int start_wnd_irp6p_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_irp6p_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6p_tfg_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wind_irp6p_tfg_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wind_irp6p_tfg_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wind_irp6p_tfg_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_irp6p_tfg_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_sarkofag.cc */
int close_wind_sarkofag_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int close_wnd_sarkofag_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_sarkofag_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_sarkofag_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_sarkofag_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int EDP_sarkofag_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_sarkofag_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wind_sarkofag_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int sarkofag_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int sarkofag_execute_motor_motion(void);
int sarkofag_execute_joint_motion(void);
int start_wnd_sarkofag_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_sarkofag_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int sarkofag_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wind_sarkofag_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wind_sarkofag_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int wind_sarkofag_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wnd_sarkofag_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_shead.cc */
int EDP_shead_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_shead_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_shead_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_smb.cc */
int EDP_smb_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_smb_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int
EDP_smb_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_speaker.cc */
int close_wnd_speaker_play(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int start_wind_speaker_play(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int clear_wind_speaker_play_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_speaker_play(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int speaker_preset_sound_play(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int speaker_play_exec(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int speaker_check_state(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_speaker_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_speaker_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int pulse_reader_speaker_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_speaker_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_reader_speaker_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int pulse_ecp_speaker(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_spkm.cc */
int EDP_spkm_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_spkm_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_spkm_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../fun_r_bird_hand->cc */

int EDP_bird_hand_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_bird_hand_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int EDP_bird_hand_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int execute_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int copy_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int start_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo);

int close_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int clear_wnd_bird_hand_command_and_status(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int execute_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int copy_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int init_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int start_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo);

int close_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

int clear_wnd_bird_hand_configuration(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../timer.cc */
int OnTimer(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../ui_init.cc */
void catch_signal(int sig);

int init(PtWidget_t *link_instance, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../wndFileLocationEvents.cc */
int FLbtnAcceptFile(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FLbtnExit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../wndForceControlEvents.cc */
int FCwndForceControlRealised(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCCreateConnection(void);
int FCRefreshPosition(void);
int FCTimerTick(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnCalibrateSensor(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnAddMacrostep(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnNewTrajectory(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnSaveTrajectory(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnExit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnOnOffReader(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnChangeExternalMotorControl(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
void SendMoveCommand(int move_type);
int FCbtnMove0Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove1Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove2Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove3Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove4Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove5Left(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove0Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove1Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove2Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove3Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove4Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int FCbtnMove5Right(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);

/* ../wndTrajectoryReproduceEvents.cc */
void SetButtonState(PtWidget_t *widget, short active);
int TRbtnStart(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnPause(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnStop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnExit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnPositionZero(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnLoadTrajectory(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int
TRbtnSaveAll(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRConnect(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRRefreshWindow(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnDSSCalibrate(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnFSCalibrate(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRbtnTryAgain(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int TRDangerousForceDetected(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
