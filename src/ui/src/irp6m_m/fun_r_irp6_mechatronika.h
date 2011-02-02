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

int irp6m_move_to_preset_position_2(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_move_to_preset_position_1(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_move_to_preset_position_0(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
int irp6m_move_to_synchro_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo);
