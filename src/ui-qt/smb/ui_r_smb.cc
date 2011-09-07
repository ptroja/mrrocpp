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

int UiRobot::synchronise()

{

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name) :
		common::UiRobot(_interface, _robot_name, lib::smb::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{

}

int UiRobot::manage_interface()
{
//	MainWindow *mw = interface.get_main_window();
	common::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case -1:

			break;
		case 0:

			break;
		case 1:
		case 2:

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

	robot_menu->setTitle(QApplication::translate("MainWindow", "S&mb", 0, QApplication::UnicodeUTF8));
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

