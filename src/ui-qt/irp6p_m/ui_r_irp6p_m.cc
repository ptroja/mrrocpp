/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "../irp6_m/wgt_irp6_m_joints.h"
#include "../irp6_m/wgt_irp6_m_motors.h"
#include "../irp6_m/wgt_irp6_m_euler.h"
#include "../irp6_m/wgt_irp6_m_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_relative_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_euler.h"

#include "ui_r_irp6p_m.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

namespace mrrocpp {
namespace ui {
namespace irp6p_m {
const std::string WGT_IRP6P_M_JOINTS = "WGT_IRP6P_M_JOINTS";
const std::string WGT_IRP6P_M_MOTORS = "WGT_IRP6P_M_MOTORS";
const std::string WGT_IRP6P_M_ANGLE_AXIS = "WGT_IRP6P_M_ANGLE_AXIS";
const std::string WGT_IRP6P_M_EULER = "WGT_IRP6P_M_EULER";
const std::string WGT_IRP6P_M_RELATIVE_ANGLE_AXIS = "WGT_IRP6P_M_RELATIVE_ANGLE_AXIS";
const std::string WGT_IRP6P_M_TOOL_ANGLE_AXIS = "WGT_IRP6P_M_TOOL_ANGLE_AXIS";
const std::string WGT_IRP6P_M_TOOL_EULER = "WGT_IRP6P_M_TOOL_EULER";
//
//
// KLASA UiRobot
//
//


void UiRobot::edp_create()
{
	if (state.edp.state == 0) {
		create_thread();

		eb.command(boost::bind(&ui::irp6p_m::UiRobot::edp_create_int, &(*this)));
	}
}

int UiRobot::edp_create_int()

{
	interface.set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota irp6_postument
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
				interface.ui_msg->message(lib::NON_FATAL_ERROR, "edp_irp6_postument already exists");
			} else if (interface.check_node_existence(state.edp.node_name, "edp_irp6_postument")) {
				state.edp.node_nr = interface.config->return_node_number(state.edp.node_name);

				{
					boost::unique_lock <boost::mutex> lock(interface.process_creation_mtx);

					ui_ecp_robot = new ui::common::EcpRobot(interface, lib::irp6p_m::ROBOT_NAME);
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
	wgt_motors->synchro_depended_init();
	return 1;

}

int UiRobot::move_to_synchro_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = 0.0;
	}
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::execute_motor_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_front_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.front_position[i];
	}
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.preset_position[variant][i];
	}
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::irp6p_m::UiRobot::synchronise_int, &(*this)));

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
			irp6_m::UiRobot(_interface, lib::irp6p_m::EDP_SECTION, lib::irp6p_m::ECP_SECTION, lib::irp6p_m::ROBOT_NAME, lib::irp6p_m::NUM_OF_SERVOS, "is_irp6p_m_active")
{
	wgt_joints = new wgt_irp6_m_joints("Irp6p_m joints", interface, *this, interface.get_main_window());
	wgt_motors = new wgt_irp6_m_motors("Irp6p_m motors", interface, *this, interface.get_main_window());
	wgt_angle_axis = new wgt_irp6_m_angle_axis("Irp6p_m angle axis", interface, *this, interface.get_main_window());
	wgt_euler = new wgt_irp6_m_euler("Irp6p_m euler", interface, *this, interface.get_main_window());
	wgt_relative_angle_axis
			= new wgt_irp6_m_relative_angle_axis("Irp6p_m relative angle axis", interface, *this, interface.get_main_window());
	wgt_tool_angle_axis
			= new wgt_irp6_m_tool_angle_axis("Irp6p_m tool angle axis", interface, *this, interface.get_main_window());
	wgt_tool_euler = new wgt_irp6_m_tool_euler("Irp6p_m tool euler", interface, *this, interface.get_main_window());

	wndbase_m[WGT_IRP6P_M_JOINTS] = wgt_joints->dwgt;
	wndbase_m[WGT_IRP6P_M_MOTORS] = wgt_motors->dwgt;
	wndbase_m[WGT_IRP6P_M_ANGLE_AXIS] = wgt_angle_axis->dwgt;
	wndbase_m[WGT_IRP6P_M_EULER] = wgt_euler->dwgt;
	wndbase_m[WGT_IRP6P_M_RELATIVE_ANGLE_AXIS] = wgt_relative_angle_axis->dwgt;
	wndbase_m[WGT_IRP6P_M_TOOL_ANGLE_AXIS] = wgt_tool_angle_axis->dwgt;
	wndbase_m[WGT_IRP6P_M_TOOL_EULER] = wgt_tool_euler->dwgt;
}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();
	Ui::MainWindow *ui = mw->get_ui();

	switch (state.edp.state)
	{

		case -1:
			mw->enable_menu_item(false, 1, ui->menuIrp6p_m);
			break;
		case 0:
			mw->enable_menu_item(false, 5, ui->menuirp6p_m_Pre_Synchro_Moves, ui->menuirp6p_m_Absolute_Moves, ui->menuirp6p_m_Preset_Positions, ui->menuirp6p_m_Relative_Moves, ui->menuirp6p_m_Tool);
			mw->enable_menu_item(false, 1, ui->actionirp6p_m_EDP_Unload);
			mw->enable_menu_item(true, 1, ui->menuIrp6p_m);
			mw->enable_menu_item(true, 1, ui->actionirp6p_m_EDP_Load);
			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, ui->menuIrp6p_m);
			mw->enable_menu_item(true, 1, ui->actionall_EDP_Unload);

			//ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, ui->menuirp6p_m_Pre_Synchro_Moves);
				mw->enable_menu_item(true, 1, ui->menuall_Preset_Positions);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 4, ui->menuirp6p_m_Absolute_Moves, ui->menuirp6p_m_Preset_Positions, ui->menuirp6p_m_Relative_Moves, ui->menuirp6p_m_Tool);
						mw->enable_menu_item(true, 1, ui->actionirp6p_m_EDP_Unload);
						mw->enable_menu_item(false, 1, ui->actionirp6p_m_EDP_Load);

						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 4, ui->menuirp6p_m_Absolute_Moves, ui->menuirp6p_m_Preset_Positions, ui->menuirp6p_m_Relative_Moves, ui->menuirp6p_m_Tool);
						mw->enable_menu_item(false, 2, ui->actionirp6p_m_EDP_Unload, ui->actionirp6p_m_EDP_Load);

						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 4, ui->menuirp6p_m_Absolute_Moves, ui->menuirp6p_m_Preset_Positions, ui->menuirp6p_m_Relative_Moves, ui->menuirp6p_m_Tool);

						break;
					default:
						break;
				}

			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, ui->menuirp6p_m_Pre_Synchro_Moves);
				mw->enable_menu_item(true, 1, ui->actionirp6p_m_EDP_Unload);
				mw->enable_menu_item(false, 1, ui->actionirp6p_m_EDP_Load);

			}
			break;
		default:
			break;

	}

	return 1;

}

// aktualizacja ustawien przyciskow
void UiRobot::process_control_window_irp6p_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{

	if (state.edp.state <= 0) {// edp wylaczone
		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_trigger);
		 */
	} else if (state.edp.state == 1) {// edp wlaczone reader czeka na start
		wlacz_PtButton_wnd_processes_control_all_reader_start = true;
		/* TR
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_trigger);
		 */
	} else if (state.edp.state == 2) {// edp wlaczone reader czeka na stop
		wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
		wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;
		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_start);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_stop);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_trigger);
		 */
	}

	state.edp.last_state = state.edp.state;
}

}
} //namespace ui
} //namespace mrrocpp
