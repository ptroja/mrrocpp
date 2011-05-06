/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_conveyor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "robot/conveyor/const_conveyor.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/wgt_single_motor_move.h"

namespace mrrocpp {
namespace ui {
namespace conveyor {
const std::string WGT_CONVEYOR_MOVE = "WGT_CONVEYOR_MOVE";
// extern ui_state_def ui_state;

//
//
// KLASA UiRobotIrp6ot_m
//
//

int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->ecp->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);

}

int UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::common::EcpRobot(interface, lib::conveyor::ROBOT_NAME);
	return 1;
}

int UiRobot::edp_create_int_extra_operations()
{
	wgt_move->synchro_depended_init();
	return 1;
}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::conveyor::UiRobot::synchronise_int, &(*this)));

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
	wgt_move->synchro_depended_init();
	wgt_move->init_and_copy();
	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
			single_motor::UiRobot(_interface, lib::conveyor::EDP_SECTION, lib::conveyor::ECP_SECTION, lib::conveyor::ROBOT_NAME, lib::conveyor::NUM_OF_SERVOS)
{

	wgt_move = new wgt_single_motor_move("Conveyor moves", interface, *this, interface.get_main_window());
	wndbase_m[WGT_CONVEYOR_MOVE] = wgt_move->dwgt;
}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();
	Ui::MainWindow *ui = mw->get_ui();

	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, ui->menuConveyor);

			break;
		case 0:
			mw->enable_menu_item(false, 3, ui->actionconveyor_EDP_Unload, ui->actionconveyor_Synchronization, ui->actionconveyor_Move);
			mw->enable_menu_item(false, 1, ui->menuconveyor_Preset_Positions);
			mw->enable_menu_item(true, 1, ui->menuConveyor);
			mw->enable_menu_item(true, 1, ui->actionconveyor_EDP_Load);

			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, ui->menuConveyor);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, ui->actionconveyor_Synchronization);
				mw->enable_menu_item(true, 1, ui->menuall_Preset_Positions);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 2, ui->actionconveyor_EDP_Unload, ui->actionconveyor_Move);
						mw->enable_menu_item(true, 1, ui->menuconveyor_Preset_Positions);
						mw->enable_menu_item(false, 1, ui->actionconveyor_EDP_Load);

						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 1, ui->actionconveyor_Move);
						mw->enable_menu_item(true, 1, ui->menuconveyor_Preset_Positions);
						mw->enable_menu_item(false, 1, ui->actionconveyor_EDP_Load, ui->actionconveyor_EDP_Unload);

						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(true, 1, ui->actionconveyor_Move);
						mw->enable_menu_item(true, 1, ui->menuconveyor_Preset_Positions);

						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 3, ui->actionconveyor_EDP_Unload, ui->actionconveyor_Synchronization, ui->actionconveyor_Move);
				mw->enable_menu_item(false, 1, ui->actionconveyor_EDP_Load);

			}
			break;
		default:
			break;
	}

	return 1;
}

// aktualizacja ustawien przyciskow
int UiRobot::process_control_window_conveyor_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{

	if (state.edp.state <= 0) {// edp wylaczone
		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_trigger);
		 */
	} else if (state.edp.state == 1) {// edp wlaczone reader czeka na start

		wlacz_PtButton_wnd_processes_control_all_reader_start = true;
		/* TR
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_trigger);
		 */
	} else if (state.edp.state == 2) {// edp wlaczone reader czeka na stop
		wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
		wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;
		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_start);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_stop);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_conveyor_reader_trigger);
		 */
	}

	state.edp.last_state = state.edp.state;

	return 1;

}

}
}
}
