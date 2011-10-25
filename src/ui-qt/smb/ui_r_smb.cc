#include "ui_r_smb.h"
#include "ui_ecp_r_smb.h"
#include "robot/smb/const_smb.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"

namespace mrrocpp {
namespace ui {
namespace smb {

//
//
// KLASA UiRobotIrp6ot_m
//
//

int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->the_robot->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);

}

UiRobot::UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name) :
		common::UiRobot(_interface, _robot_name, lib::smb::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{

}

int UiRobot::manage_interface()
{

	common::UiRobot::manage_interface();

	switch (state.edp.state)
	{

		case common::UI_EDP_INACTIVE:

			break;
		case common::UI_EDP_OFF:
			action_Clear_Fault->setEnabled(false);
			action_Synchronisation->setEnabled(false);
			action_command->setEnabled(false);
			break;
		case common::UI_EDP_WAITING_TO_START_READER:
		case common::UI_EDP_WAITING_TO_STOP_READER:




			action_Clear_Fault->setEnabled(true);
			action_command->setEnabled(true);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				action_Synchronisation->setEnabled(false);

				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
					case common::UI_MP_WAITING_FOR_START_PULSE:
						action_command->setEnabled(true);
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						action_command->setEnabled(false);
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				action_Synchronisation->setEnabled(true);

			}
			break;
		default:
			break;
	}

	return 1;
}

void UiRobot::setup_menubar()
{

	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	robot_menu->setTitle(QApplication::translate("MainWindow", "S&mb", 0, QApplication::UnicodeUTF8));

	action_Synchronisation = new Ui::MenuBarAction(QString("&Synchronisation"), this, menuBar);
	action_command = new Ui::MenuBarAction(QString("&Command"), wgts[WGT_SMB_COMMAND], signalDispatcher, menuBar);
	action_Clear_Fault = new Ui::MenuBarAction(QString("&Clear Fault"), this, menuBar);

	robot_menu->addAction(action_Synchronisation);
	robot_menu->addAction(action_command);
	robot_menu->addSeparator();
	robot_menu->addAction(action_Clear_Fault);

	// connections
	connect(action_Synchronisation, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Clear_Fault, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Clear_Fault_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);

}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::smb::UiRobot::synchronise_int, &(*this)));

	return 1;

}

int UiRobot::synchronise_int()

{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota spkm

		if ((is_edp_loaded()) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->the_robot->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->the_robot->is_synchronised();
		} else {
			// 	printf("edp spkm niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_IN_ROBOT

	// modyfikacje menu
	interface.manage_interface();
	wgts[WGT_SMB_COMMAND]->synchro_depended_init();

	return 1;

}

int UiRobot::execute_clear_fault()
{
	try {

		ui_ecp_robot->clear_fault();

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

int UiRobot::execute_stop_motor()
{
	try {

		ui_ecp_robot->stop_motors();

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

void UiRobot::null_ui_ecp_robot()
{
	ui_ecp_robot = NULL;
}

}
} //namespace ui
} //namespace mrrocpp

