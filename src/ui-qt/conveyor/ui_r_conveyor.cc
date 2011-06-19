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

#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"

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
	ui_ecp_robot = new ui::common::EcpRobot(*this);
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
	CATCH_SECTION_IN_ROBOT

	// modyfikacje menu
	interface.manage_interface();
	wgt_move->synchro_depended_init();
	wgt_move->init_and_copy();
	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
	single_motor::UiRobot(_interface, lib::conveyor::ROBOT_NAME, lib::conveyor::NUM_OF_SERVOS)
{

	wgt_move = new wgt_single_motor_move("Conveyor moves", interface, *this, interface.get_main_window());
	wndbase_m[WGT_CONVEYOR_MOVE] = wgt_move->dwgt;
}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();

	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, robot_menu);

			break;
		case 0:
			mw->enable_menu_item(false, 3, EDP_Unload, actionconveyor_Synchronization, actionconveyor_Move);
			mw->enable_menu_item(false, 1, menuconveyor_Preset_Positions);
			mw->enable_menu_item(true, 1, robot_menu);
			mw->enable_menu_item(true, 1, EDP_Load);

			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, robot_menu);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, actionconveyor_Synchronization);
				mw->enable_menu_item(true, 1, mw->getMenuBar()->menuall_Preset_Positions);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 2, EDP_Unload, actionconveyor_Move);
						mw->enable_menu_item(true, 1, menuconveyor_Preset_Positions);
						mw->enable_menu_item(false, 1, EDP_Load);
						block_ecp_trigger();
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 1, actionconveyor_Move);
						mw->enable_menu_item(true, 1, menuconveyor_Preset_Positions);
						mw->enable_menu_item(false, 1, EDP_Load, EDP_Unload);
						block_ecp_trigger();
						break;
					case common::UI_MP_TASK_RUNNING:
						unblock_ecp_trigger();
						break;
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(true, 1, actionconveyor_Move);
						mw->enable_menu_item(true, 1, menuconveyor_Preset_Positions);
						block_ecp_trigger();
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 3, EDP_Unload, actionconveyor_Synchronization, actionconveyor_Move);
				mw->enable_menu_item(false, 1, EDP_Load);

			}
			break;
		default:
			break;
	}

	return 1;
}

void UiRobot::make_connections()
{
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	connect(actionconveyor_Move, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)),	Qt::AutoCompatConnection);
	connect(actionconveyor_Synchronization, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Move_triggered(mrrocpp::ui::common::UiRobot*)),				Qt::AutoCompatConnection);
	connect(actionconveyor_Synchro_Position,SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)),	Qt::AutoCompatConnection);
	connect(actionconveyor_Position_0, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionconveyor_Position_1, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionconveyor_Position_2, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
}

void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

    actionconveyor_Synchronization 		= new Ui::MenuBarAction(QString("&Synchronization"),this, menuBar);
    actionconveyor_Move					= new Ui::MenuBarAction(QString("&Move"),this, menuBar);
    actionconveyor_Synchro_Position 	= new Ui::MenuBarAction(QString("&Synchro Position"),this, menuBar);
    actionconveyor_Position_0 			= new Ui::MenuBarAction(QString("Position &0"),this, menuBar);
    actionconveyor_Position_1 			= new Ui::MenuBarAction(QString("Position &1"),this, menuBar);
    actionconveyor_Position_2			= new Ui::MenuBarAction(QString("Position &2"),this, menuBar);

    menuconveyor_Preset_Positions = new QMenu(robot_menu);

	robot_menu->addSeparator();
	robot_menu->addAction(actionconveyor_Synchronization);
	robot_menu->addAction(actionconveyor_Move);
	robot_menu->addAction(menuconveyor_Preset_Positions->menuAction());
	menuconveyor_Preset_Positions->addAction(actionconveyor_Synchro_Position);
	menuconveyor_Preset_Positions->addAction(actionconveyor_Position_0);
	menuconveyor_Preset_Positions->addAction(actionconveyor_Position_1);
	menuconveyor_Preset_Positions->addAction(actionconveyor_Position_2);

    robot_menu->setTitle(QApplication::translate("MainWindow", "&Conveyor", 0, QApplication::UnicodeUTF8));
    menuconveyor_Preset_Positions->setTitle(QApplication::translate("MainWindow", "&Preset Positions", 0, QApplication::UnicodeUTF8));
    make_connections();
}



// aktualizacja ustawien przyciskow
int UiRobot::process_control_window_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
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
