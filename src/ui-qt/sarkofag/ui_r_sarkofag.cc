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
	MainWindow *mw = interface.get_main_window();
	single_motor::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case -1:

			break;
		case 0:
			mw->enable_menu_item(false, 2, actionsarkofag_Move, actionsarkofag_Servo_Algorithm);

			break;
		case 1:
		case 2:

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {

				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 2, actionsarkofag_Move, actionsarkofag_Servo_Algorithm);

						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 2, actionsarkofag_Move, actionsarkofag_Servo_Algorithm);

						break;
					case common::UI_MP_TASK_RUNNING:

						break;
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 2, actionsarkofag_Move, actionsarkofag_Servo_Algorithm);
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, actionsarkofag_Move);

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

	connect(actionsarkofag_Synchronisation, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
//	connect(actionsarkofag_Move, 				SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Move_triggered(mrrocpp::ui::common::UiRobot*)),				Qt::AutoCompatConnection);
	connect(actionsarkofag_Synchro_Position, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionsarkofag_Front_Position, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Front_Position_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionsarkofag_Position_0, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionsarkofag_Position_1, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionsarkofag_Position_2, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
//	connect(actionsarkofag_Servo_Algorithm, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(), 		Qt::AutoCompatConnection);

}

void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	actionsarkofag_Synchronisation = new Ui::MenuBarAction(QString("&Synchronization"), this, menuBar);
	actionsarkofag_Move = new Ui::MenuBarAction(QString("&Move"), wgts[WGT_SARKOFAG_MOVE], signalDispatcher, menuBar);
	actionsarkofag_Synchro_Position = new Ui::MenuBarAction(QString("&Synchro position"), this, menuBar);
	actionsarkofag_Front_Position = new Ui::MenuBarAction(QString("&Front Position"), this, menuBar);
	actionsarkofag_Position_0 = new Ui::MenuBarAction(QString("Position &0"), this, menuBar);
	actionsarkofag_Position_1 = new Ui::MenuBarAction(QString("Position &1"), this, menuBar);
	actionsarkofag_Position_2 = new Ui::MenuBarAction(QString("Position &2"), this, menuBar);
	actionsarkofag_Servo_Algorithm = new Ui::MenuBarAction(QString("S&ervo Algorithm"), this, menuBar);

	menusarkofag_Preset_Positions = new QMenu(robot_menu);

	robot_menu->addSeparator();
	robot_menu->addAction(actionsarkofag_Synchronisation);
	robot_menu->addAction(actionsarkofag_Move);
	robot_menu->addAction(menusarkofag_Preset_Positions->menuAction());
	robot_menu->addAction(actionsarkofag_Servo_Algorithm);
	menusarkofag_Preset_Positions->addAction(actionsarkofag_Synchro_Position);
	menusarkofag_Preset_Positions->addAction(actionsarkofag_Front_Position);
	menusarkofag_Preset_Positions->addAction(actionsarkofag_Position_0);
	menusarkofag_Preset_Positions->addAction(actionsarkofag_Position_1);
	menusarkofag_Preset_Positions->addAction(actionsarkofag_Position_2);

	robot_menu->setTitle(QApplication::translate("MainWindow", "&Sarkofag", 0, QApplication::UnicodeUTF8));
	menusarkofag_Preset_Positions->setTitle(QApplication::translate("MainWindow", "Pr&eset Positions", 0, QApplication::UnicodeUTF8));
	make_connections();
}

}
} //namespace ui
} //namespace mrrocpp

