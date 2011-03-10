/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "wgt_irp6ot_m_joints.h"

#include "ui_r_irp6ot_m.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

namespace mrrocpp {
namespace ui {
namespace irp6ot_m {

//
//
// KLASA UiRobot
//
//
void UiRobot::edp_create()
{
	if (state.edp.state == 0) {
		create_thread();

		eb.command(boost::bind(&ui::irp6ot_m::UiRobot::edp_create_int, &(*this)));
	}
}

int UiRobot::edp_create_int()

{

	interface.set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota irp6_on_track
		if (state.edp.state == 0) {

			state.edp.state = 0;
			state.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += state.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string += state.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if (((!(state.edp.test_mode)) && (access(tmp_string.c_str(), R_OK) == 0))
					|| (access(tmp2_string.c_str(), R_OK) == 0)) {
				interface.ui_msg->message(lib::NON_FATAL_ERROR, "edp_irp6_on_track already exists");
			} else if (interface.check_node_existence(state.edp.node_name, "edp_irp6_on_track")) {

				state.edp.node_nr = interface.config->return_node_number(state.edp.node_name);

				{
					boost::unique_lock <boost::mutex> lock(interface.process_creation_mtx);

					ui_ecp_robot = new ui::common::EcpRobot(interface, lib::irp6ot_m::ROBOT_NAME);
				}

				state.edp.pid = ui_ecp_robot->ecp->get_EDP_pid();

				if (state.edp.pid < 0) {

					state.edp.state = 0;
					fprintf(stderr, "edp spawn failed: %s\n", strerror(errno));
					delete ui_ecp_robot;
				} else { // jesli spawn sie powiodl

					state.edp.state = 1;

					connect_to_reader();

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;

					ui_ecp_robot->get_controller_state(robot_controller_initial_state_tmp);

					//state.edp.state = 1; // edp wlaczone reader czeka na start

					state.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try

	CATCH_SECTION_UI

	interface.manage_interface();

	return 1;

}

int UiRobot::move_to_synchro_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = 0.0;
	}
	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_motor_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_front_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.front_position[i];
	}

	//	printf(" move_to_front_position state.edp.front_position: %lf, %lf, %lf, %lf, %lf, %lf\n ", state.edp.front_position[0], state.edp.front_position[1], state.edp.front_position[2], state.edp.front_position[3], state.edp.front_position[4], state.edp.front_position[5]);

	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.preset_position[variant][i];
	}
	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::execute_motor_motion()
{
	try {

		ui_ecp_robot->move_motors(desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int UiRobot::execute_joint_motion()
{
	try {

		ui_ecp_robot->move_joints(desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

UiRobot::UiRobot(common::Interface& _interface) :
			common::UiRobot(_interface, lib::irp6ot_m::EDP_SECTION, lib::irp6ot_m::ECP_SECTION, lib::irp6ot_m::ROBOT_NAME, lib::irp6ot_m::NUM_OF_SERVOS, "is_irp6ot_m_active"),
			is_wind_irp6ot_int_open(false), is_wind_irp6ot_inc_open(false), is_wind_irp6ot_xyz_euler_zyz_open(false),
			is_wind_irp6ot_xyz_angle_axis_open(false), is_wind_irp6ot_xyz_aa_relative_open(false),
			is_wind_irp6ot_xyz_angle_axis_ts_open(false), is_wind_irp6ot_xyz_euler_zyz_ts_open(false),
			is_wind_irp6ot_kinematic_open(false), is_wind_irp6ot_servo_algorithm_open(false), ui_ecp_robot(NULL)
{
	wgt_joints = new wgt_irp6ot_m_joints(interface, *this, interface.get_main_window());
	wndbase_m[WGT_IRP6OT_JOINTS] = wgt_joints->dwgt;

}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::synchronise_int, &(*this)));

	return 1;

}

int UiRobot::synchronise_int()

{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota irp6_on_track

		if ((state.edp.state > 0) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->ecp->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp irp6_on_track niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu
	interface.manage_interface();

	return 1;

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();
	Ui::MainWindow *ui = mw->get_ui();

	switch (state.edp.state)
	{

		case -1:
			mw->enable_menu_item(false, 1, ui->menuIrp6ot_m);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track, NULL);
			 */
			break;
		case 0:
			mw->enable_menu_item(false, 1, ui->actionirp6ot_m_EDP_Unload); //??? brakuje czegos?
			mw->enable_menu_item(false, 5, ui->menuirp6ot_m_Pre_Synchro_Moves, ui->menuirp6ot_m_Absolute_moves, ui->menuIrp6ot_m_Relative_Moves, ui->menuirp6ot_m_Tool, ui->menuirp6ot_m_Preset_Positions);
			mw->enable_menu_item(true, 1, ui->menuIrp6ot_m);
			mw->enable_menu_item(true, 1, ui->actionirp6ot_m_EDP_Load);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_edp_unload, ABN_mm_irp6_on_track_pre_synchro_moves, ABN_mm_irp6_on_track_absolute_moves,
			 ABN_mm_irp6_on_track_relative_moves, ABN_mm_irp6_on_track_tool_specification, ABN_mm_irp6_on_track_preset_positions, ABN_mm_irp6_on_track_kinematic, ABN_mm_irp6_on_track_servo_algorithm, NULL);
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_on_track, ABN_mm_irp6_on_track_edp_load, NULL);
			 */
			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, ui->menuIrp6ot_m);
			mw->enable_menu_item(true, 1, ui->actionall_EDP_Unload);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_on_track, NULL);
			 //ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			 */

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, ui->menuirp6ot_m_Pre_Synchro_Moves);
				mw->enable_menu_item(true, 1, ui->menuall_Preset_Positions);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_pre_synchro_moves, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);
				 */
				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 4, ui->menuirp6ot_m_Absolute_moves, ui->menuIrp6ot_m_Relative_Moves, ui->menuirp6ot_m_Tool, ui->menuirp6ot_m_Preset_Positions);
						mw->enable_menu_item(true, 1, ui->actionirp6ot_m_EDP_Unload);
						mw->enable_menu_item(false, 1, ui->actionirp6ot_m_EDP_Load);
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_on_track_edp_unload, ABN_mm_irp6_on_track_absolute_moves,
						 ABN_mm_irp6_on_track_relative_moves, ABN_mm_irp6_on_track_tool_specification, ABN_mm_irp6_on_track_preset_positions, ABN_mm_irp6_on_track_kinematic, ABN_mm_irp6_on_track_servo_algorithm, NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_edp_load, NULL);
						 */
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 4, ui->menuirp6ot_m_Absolute_moves, ui->menuIrp6ot_m_Relative_Moves, ui->menuirp6ot_m_Tool, ui->menuirp6ot_m_Preset_Positions);
						mw->enable_menu_item(false, 2, ui->actionirp6ot_m_EDP_Load, ui->actionirp6ot_m_EDP_Unload);
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_on_track_absolute_moves, ABN_mm_irp6_on_track_relative_moves, ABN_mm_irp6_on_track_tool_specification,
						 ABN_mm_irp6_on_track_preset_positions, ABN_mm_irp6_on_track_kinematic, ABN_mm_irp6_on_track_servo_algorithm, NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_edp_load, ABN_mm_irp6_on_track_edp_unload, NULL);
						 */
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 4, ui->menuirp6ot_m_Absolute_moves, ui->menuIrp6ot_m_Relative_Moves, ui->menuirp6ot_m_Tool, ui->menuirp6ot_m_Preset_Positions);
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						 ABN_mm_irp6_on_track_absolute_moves, ABN_mm_irp6_on_track_relative_moves, ABN_mm_irp6_on_track_preset_positions, ABN_mm_irp6_on_track_tool_specification,
						 ABN_mm_irp6_on_track_kinematic, ABN_mm_irp6_on_track_servo_algorithm, NULL);
						 */
						break;
					default:
						break;
				}

			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 2, ui->actionirp6ot_m_EDP_Unload, ui->actionall_Synchronisation);
				mw->enable_menu_item(true, 1, ui->menuirp6p_m_Pre_Synchro_Moves);
				mw->enable_menu_item(false, 1, ui->actionirp6ot_m_EDP_Load);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_on_track_edp_unload, ABN_mm_irp6_on_track_pre_synchro_moves, NULL);
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_on_track_edp_load, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
				 */
			}
			break;
		default:
			break;

	}

	return 1;
}

// aktualizacja ustawien przyciskow
int UiRobot::process_control_window_irp6ot_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{

	if (state.edp.state <= 0) {// edp wylaczone
		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		 */
	} else {
		if (state.edp.state == 1) {// edp wlaczone reader czeka na start
			wlacz_PtButton_wnd_processes_control_all_reader_start = true;
			/* TR
			 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
			 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
			 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
			 */
		} else if (state.edp.state == 2) {// edp wlaczone reader czeka na stop
			wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
			wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;
			/* TR
			 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
			 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
			 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
			 */
		}
	}

	state.edp.last_state = state.edp.state;

	return 1;

}


void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

}
} //namespace ui
} //namespace mrrocpp
