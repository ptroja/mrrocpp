/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

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

namespace mrrocpp {
namespace ui {
namespace sbench {

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

int UiRobot::synchronise()

{

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
		common::UiRobot(_interface, lib::sbench::ROBOT_NAME, lib::sbench::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{

}

int UiRobot::manage_interface()
{
//	MainWindow *mw = interface.get_main_window();
	common::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case common::UI_EDP_INACTIVE:

			break;
		case common::UI_EDP_OFF:

			break;
		case common::UI_EDP_WAITING_TO_START_READER:
		case common::UI_EDP_WAITING_TO_STOP_READER:


			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {

				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:

						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:

						break;
					case common::UI_MP_TASK_RUNNING:

						break;
					case common::UI_MP_TASK_PAUSED:

						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{

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
	//	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

	robot_menu->setTitle(QApplication::translate("MainWindow", "S&head", 0, QApplication::UnicodeUTF8));

	robot_menu->setTitle(QApplication::translate("MainWindow", "S&bench", 0, QApplication::UnicodeUTF8));

}

void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

void UiRobot::null_ui_ecp_robot()
{
	ui_ecp_robot = NULL;

}

void UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new EcpRobot(*this);
	//	return 1;
}

}
} //namespace ui
} //namespace mrrocpp

