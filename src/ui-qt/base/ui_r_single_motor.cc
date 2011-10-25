/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"

namespace mrrocpp {
namespace ui {
namespace single_motor {

// extern ui_state_def ui_state;

//
//
// KLASA UiRobotIrp6ot_m
//
//

UiRobot::UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name, int _number_of_servos) :
		common::UiRobot(_interface, _robot_name, _number_of_servos)
{

}

void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;

}

void UiRobot::null_ui_ecp_robot()
{
	ui_ecp_robot = NULL;

}

void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	action_Synchronisation = new Ui::MenuBarAction(QString("&Synchronisation"), this, menuBar);
	action_Synchro_Position = new Ui::MenuBarAction(QString("&Synchro Position"), this, menuBar);
	action_Position_0 = new Ui::MenuBarAction(QString("Position &0"), this, menuBar);
	action_Position_1 = new Ui::MenuBarAction(QString("Position &1"), this, menuBar);
	action_Position_2 = new Ui::MenuBarAction(QString("Position &2"), this, menuBar);

	menu_Preset_Positions = new QMenu(robot_menu);
	robot_menu->addSeparator();
	robot_menu->addAction(action_Synchronisation);
	robot_menu->addAction(menu_Preset_Positions->menuAction());
	menu_Preset_Positions->addAction(action_Synchro_Position);
	menu_Preset_Positions->addAction(action_Position_0);
	menu_Preset_Positions->addAction(action_Position_1);
	menu_Preset_Positions->addAction(action_Position_2);

	menu_Preset_Positions->setTitle(QApplication::translate("MainWindow", "&Preset positions", 0, QApplication::UnicodeUTF8));

	// connections
	connect(action_Synchronisation, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Synchro_Position, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_0, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_1, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_2, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);

}

int UiRobot::manage_interface()
{

	common::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case common::UI_EDP_INACTIVE:

			break;
		case common::UI_EDP_OFF:
			menu_Preset_Positions->setEnabled(false);
			action_Synchronisation->setEnabled(false);
			break;
		case common::UI_EDP_WAITING_TO_START_READER:
		case common::UI_EDP_WAITING_TO_STOP_READER:



			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				action_Synchronisation->setEnabled(false);
				//	mw->enable_menu_item(true, 1, menuBar->menuall_Preset_Positions);

				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
					case common::UI_MP_WAITING_FOR_START_PULSE:
						menu_Preset_Positions->setEnabled(true);
						break;
					case common::UI_MP_TASK_RUNNING:

						break;
					case common::UI_MP_TASK_PAUSED:
						menu_Preset_Positions->setEnabled(false);
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

}
}
}
