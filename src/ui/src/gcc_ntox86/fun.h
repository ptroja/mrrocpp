/* ../fun.cc */
int set_ui_busy_state_notification(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int set_ui_ready_state_notification(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
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

int all_robots_move_to_synchro_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int all_robots_move_to_preset_position_1(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int all_robots_move_to_preset_position_2(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int all_robots_move_to_preset_position_0(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int all_robots_move_to_front_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
