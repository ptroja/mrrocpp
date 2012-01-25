/*!
 * @file
 * @brief File contains UiRobot class definition for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "ui_r_sbench.h"
#include "ui_ecp_r_sbench.h"
#include "robot/sbench/const_sbench.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"

#include "wgt_sbench_voltage_command.h"
#include "wgt_sbench_preasure_command.h"

namespace mrrocpp {
namespace ui {
namespace sbench {


void UiRobot::synchronise()
{
}

UiRobot::UiRobot(common::Interface& _interface) :
		common::UiRobot(_interface, lib::sbench::ROBOT_NAME, lib::sbench::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{
	add_wgt <wgt_sbench_voltage_command>(sbench::WGT_SBENCH_VOLTAGE_COMMAND, "Sbench voltage command");
	add_wgt <wgt_sbench_preasure_command>(sbench::WGT_SBENCH_PREASURE_COMMAND, "Sbench preasure command");

}

void UiRobot::manage_interface()
{
//	MainWindow *mw = interface.get_main_window();
	common::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case common::UI_EDP_INACTIVE:

			break;
		case common::UI_EDP_OFF:
			action_voltage_command->setEnabled(false);
			action_preasure_command->setEnabled(false);
			break;
		case common::UI_EDP_WAITING_TO_START_READER:
		case common::UI_EDP_WAITING_TO_STOP_READER:
			action_voltage_command->setEnabled(true);
			action_preasure_command->setEnabled(true);
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {

				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
					case common::UI_MP_WAITING_FOR_START_PULSE:
						action_voltage_command->setEnabled(true);
						action_preasure_command->setEnabled(true);
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						action_voltage_command->setEnabled(false);
						action_preasure_command->setEnabled(false);
						break;
					default:
						break;
				}
			} else {
				// jesli robot jest niezsynchronizowany
			}
			break;
		default:
			break;
	}

}

void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	robot_menu->setTitle(QApplication::translate("MainWindow", "S&bench", 0, QApplication::UnicodeUTF8));

	action_voltage_command =
			new Ui::MenuBarAction(QString("&Voltage command"), wgts[WGT_SBENCH_VOLTAGE_COMMAND], signalDispatcher, menuBar);

	action_preasure_command =
			new Ui::MenuBarAction(QString("&Preasure command"), wgts[WGT_SBENCH_PREASURE_COMMAND], signalDispatcher, menuBar);

	robot_menu->addAction(action_voltage_command);
	robot_menu->addAction(action_preasure_command);

}


void UiRobot::create_ui_ecp_robot()
{
	common::UiRobot::ui_ecp_robot = ui_ecp_robot = new EcpRobot(*this);
}

}
} //namespace ui
} //namespace mrrocpp
