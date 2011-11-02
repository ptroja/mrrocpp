/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_ecp_r_bird_hand.h"
#include "ui_r_bird_hand.h"

#include "wgt_bird_hand_command.h"
//#include "ui/src/bird_hand/wnd_bird_hand_configuration.h"

#include "robot/bird_hand/const_bird_hand.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"

//
// KLASA UiRobotBirdHand
//
//

namespace mrrocpp {
namespace ui {
namespace bird_hand {

const std::string UiRobot::WGT_COMMAND_AND_STATUS = "WGT_COMMAND_AND_STATUS";
const std::string UiRobot::WGT_CONFIGURATION = "WGT_CONFIGURATION";

int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->the_robot->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);

}

void UiRobot::edp_create()
{
	if (state.edp.state == common::UI_EDP_OFF) {
		create_thread();

		eb.command(boost::bind(&ui::bird_hand::UiRobot::edp_create_int, &(*this)));
	}
}

void UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::bird_hand::EcpRobot(*this);
//	return 1;
}

int UiRobot::synchronise()
{
	return 1;
}

UiRobot::UiRobot(common::Interface& _interface) :
		common::UiRobot(_interface, lib::bird_hand::ROBOT_NAME, lib::bird_hand::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{
	add_wgt <wgt_bird_hand_command>(WGT_COMMAND_AND_STATUS, "Birdhand command and status");
//	wndbase_m[WGT_BIRD_HAND_COMMAND] = wgts[WGT_COMMAND_AND_STATUS]->dwgt;
//	add_wgt<wgt_configuration>	(WGT_CONFIGURATION, "Birdhand configuration");
//	wndbase_m[WGT_BIRD_HAND_COMMAND] = wgts[WGT_COMMAND_AND_STATUS]->dwgt;
}

int UiRobot::manage_interface()
{

	common::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case common::UI_EDP_INACTIVE:

			break;
		case common::UI_EDP_OFF:
			actionbirdhand_Command->setEnabled(false);
			break;
		case common::UI_EDP_WAITING_TO_START_READER:
		case common::UI_EDP_WAITING_TO_STOP_READER:
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {

				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						actionbirdhand_Command->setEnabled(true);

						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						actionbirdhand_Command->setEnabled(true);
						break;
					case common::UI_MP_TASK_RUNNING:
						unblock_ecp_trigger();
						break;
					case common::UI_MP_TASK_PAUSED:
						actionbirdhand_Command->setEnabled(false);

						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				actionbirdhand_Command->setEnabled(false);

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

	actionbirdhand_Command =
			new Ui::MenuBarAction(QString("&Command"), wgts[WGT_COMMAND_AND_STATUS], signalDispatcher, menuBar);
//    actionbirdhand_Configuration 				= new Ui::MenuBarAction(QString("Co&Nfiguration"), this, menuBar);

	robot_menu->addSeparator();
	robot_menu->addAction(actionbirdhand_Command);
// 	robot_menu->addAction(actionbirdhand_Configuration);

	robot_menu->setTitle(QApplication::translate("MainWindow", "&Birdhand", 0, QApplication::UnicodeUTF8));
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
