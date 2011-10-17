/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/*W SPKM tez to jest
 #include "ui_r_spkm.h"
 #include "ui_ecp_r_spkm.h"
 #include "wgt_spkm_inc.h"
 */

#include "wgt_polycrank_int.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "ui_r_polycrank.h"
#include "robot/polycrank/const_polycrank.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"
#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"

namespace mrrocpp {
namespace ui {
namespace polycrank {

const std::string UiRobot::WGT_INT = "WGT_INT";

//
//
// KLASA UiRobot
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

UiRobot::UiRobot(common::Interface& _interface) :
		common::UiRobot(_interface, lib::polycrank::ROBOT_NAME, lib::polycrank::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{
	add_wgt <wgt_polycrank_int>(WGT_INT, "Polycrank int");

}

void UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::common::EcpRobot(*this);
//	return 1;
}

int UiRobot::synchronise()
{
	eb.command(boost::bind(&ui::polycrank::UiRobot::synchronise_int, &(*this)));
	//Polycrank will be always in synchronise poses
	return 1;

}

int UiRobot::synchronise_int()
{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota polycrank

		if ((is_edp_loaded()) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->ecp->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp polycrank niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_IN_ROBOT

	// modyfikacje menu
	interface.manage_interface();

	return 1;

}

int UiRobot::manage_interface()
{
	//MainWindow *mw = interface.get_main_window();
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
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	actionpolycrank_Move_Joints =
			new Ui::MenuBarAction(QString("Move &Joints"), wgts[WGT_INT], signalDispatcher, menuBar);

	robot_menu->addSeparator();
	robot_menu->addAction(actionpolycrank_Move_Joints);

	menuBar->menuRobot->addAction(robot_menu->menuAction());
	robot_menu->setTitle(QApplication::translate("MainWindow", "Polyc&rank", 0, QApplication::UnicodeUTF8));

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
