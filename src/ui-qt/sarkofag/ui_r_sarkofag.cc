/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_sarkofag.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "robot/sarkofag/const_sarkofag.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/wgt_single_motor_move.h"

#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"

namespace mrrocpp {
namespace ui {
namespace sarkofag {
const std::string WGT_SARKOFAG_MOVE = "WGT_SARKOFAG_MOVE";
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

void UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::common::EcpRobot(*this);
	//return 1;
}

int UiRobot::edp_create_int_extra_operations()
{
	wgts[WGT_SARKOFAG_MOVE]->synchro_depended_init();
	return 1;
}

int UiRobot::execute_motor_motion()
{
	try {

		ui_ecp_robot->move_motors(desired_pos);

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

int UiRobot::execute_joint_motion()
{
	try {

		ui_ecp_robot->move_joints(desired_pos);

	} // end try
	CATCH_SECTION_IN_ROBOT

	return 1;
}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::sarkofag::UiRobot::synchronise_int, &(*this)));

	return 1;

}

int UiRobot::synchronise_int()

{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota irp6_on_track

		if ((is_edp_loaded()) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->ecp->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp irp6_on_track niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_IN_ROBOT

	// modyfikacje menu
	interface.manage_interface();
	wgts[WGT_SARKOFAG_MOVE]->synchro_depended_init();
	wgts[WGT_SARKOFAG_MOVE]->init_and_copy();
	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
		single_motor::UiRobot(_interface, lib::sarkofag::ROBOT_NAME, lib::sarkofag::NUM_OF_SERVOS)
{
	add_wgt <wgt_single_motor_move>(WGT_SARKOFAG_MOVE, "Sarkofag moves");
}

int UiRobot::manage_interface()
{

	single_motor::UiRobot::manage_interface();

	switch (state.edp.state)
	{

		case common::UI_EDP_INACTIVE:

			break;
		case common::UI_EDP_OFF:
			actionsarkofag_Move->setEnabled(false);
			break;
		case common::UI_EDP_WAITING_TO_START_READER:
		case common::UI_EDP_WAITING_TO_STOP_READER:

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:

					case common::UI_MP_WAITING_FOR_START_PULSE:
						actionsarkofag_Move->setEnabled(true);

						break;
					case common::UI_MP_TASK_RUNNING:

						break;
					case common::UI_MP_TASK_PAUSED:
						actionsarkofag_Move->setEnabled(false);
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				actionsarkofag_Move->setEnabled(true);
			}
			break;
		default:
			break;
	}

	return 1;
}

void UiRobot::setup_menubar()
{
	single_motor::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	actionsarkofag_Move = new Ui::MenuBarAction(QString("&Move"), wgts[WGT_SARKOFAG_MOVE], signalDispatcher, menuBar);
	robot_menu->addAction(actionsarkofag_Move);

	robot_menu->setTitle(QApplication::translate("MainWindow", "&Sarkofag", 0, QApplication::UnicodeUTF8));
}

}
} //namespace ui
} //namespace mrrocpp

