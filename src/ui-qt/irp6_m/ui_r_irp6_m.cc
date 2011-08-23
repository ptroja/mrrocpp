/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "../irp6_m/wgt_irp6_m_motors.h"
#include "../irp6_m/wgt_irp6_m_joints.h"

#include "ui_r_irp6_m.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"

#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"
#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"

namespace mrrocpp {
namespace ui {
namespace irp6_m {

const std::string UiRobot::WGT_JOINTS = "WGT_JOINTS";
const std::string UiRobot::WGT_MOTORS = "WGT_MOTORS";
const std::string UiRobot::WGT_ANGLE_AXIS = "WGT_ANGLE_AXIS";
const std::string UiRobot::WGT_EULER = "WGT_EULER";
const std::string UiRobot::WGT_RELATIVE_ANGLE_AXIS = "WGT_RELATIVE_ANGLE_AXIS";
const std::string UiRobot::WGT_TOOL_ANGLE_AXIS = "WGT_TOOL_ANGLE_AXIS";
const std::string UiRobot::WGT_TOOL_EULER = "WGT_TOOL_EULER";
//
//
// KLASA UiRobot
//
//
int UiRobot::synchronise()
{
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
	wgts[WGT_MOTORS]->synchro_depended_init();
	wgts[WGT_MOTORS]->init_and_copy();

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name, int _number_of_servos) :
		common::UiRobot(_interface, _robot_name, _number_of_servos), ui_ecp_robot(NULL)
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

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();
//	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

	common::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case -1:
			//	mw->enable_menu_item(false, 1, robot_menu);
			break;
		case 0:
			mw->enable_menu_item(false, 1, menu_Preset_Positions);
			mw->enable_menu_item(false, 1, action_Synchronisation);
			//	mw->enable_menu_item(true, 1, robot_menu);
			//	mw->enable_menu_item(true, 1, EDP_Load);
			mw->enable_menu_item(false, 4, menu_Pre_Synchro_Moves, menu_Absolute_Moves, menu_Relative_Moves, menu_Tool);

			break;
		case 1:
		case 2:
			//	mw->enable_menu_item(true, 1, robot_menu);
			//	mw->enable_menu_item(true, 1, menuBar->actionall_EDP_Unload);
			mw->enable_menu_item(false, 1, menu_Pre_Synchro_Moves);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, action_Synchronisation);
				//	mw->enable_menu_item(true, 1, menuBar->menuall_Preset_Positions);

				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 1, menu_Preset_Positions);
						//	mw->enable_menu_item(true, 1, EDP_Unload);
						//	mw->enable_menu_item(false, 1, EDP_Load);
						mw->enable_menu_item(true, 3, menu_Absolute_Moves, menu_Relative_Moves, menu_Tool);
						//	block_ecp_trigger();
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 1, menu_Preset_Positions);
						//	mw->enable_menu_item(false, 2, EDP_Load, EDP_Unload);
						mw->enable_menu_item(true, 3, menu_Absolute_Moves, menu_Relative_Moves, menu_Tool);
						//	block_ecp_trigger();
						break;
					case common::UI_MP_TASK_RUNNING:
						//	unblock_ecp_trigger();
						break;
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 1, menu_Preset_Positions);
						mw->enable_menu_item(false, 3, menu_Absolute_Moves, menu_Relative_Moves, menu_Tool);
						//	block_ecp_trigger();
						break;
					default:
						break;
				}

			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, action_Synchronisation);
				//	mw->enable_menu_item(false, 1, EDP_Load);
				mw->enable_menu_item(true, 1, menu_Pre_Synchro_Moves);
			}
			break;
		default:
			break;

	}

	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{

	return 1;
}

void UiRobot::make_connections()
{

	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	connect(action_Synchronisation, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Synchro_Position, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Front_Position, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Front_Position_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_0, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_1, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_2, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
}

void UiRobot::setup_menubar()
{

	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	action_Synchronisation = new Ui::MenuBarAction(QString("&Synchronisation"), this, menuBar);
	action_Synchro_Position = new Ui::MenuBarAction(QString("&Synchro Position"), this, menuBar);
	action_Front_Position = new Ui::MenuBarAction(QString("&Front Position"), this, menuBar);
	action_Position_0 = new Ui::MenuBarAction(QString("Position &0"), this, menuBar);
	action_Position_1 = new Ui::MenuBarAction(QString("Position &1"), this, menuBar);
	action_Position_2 = new Ui::MenuBarAction(QString("Position &2"), this, menuBar);

	action_Pre_Synchro_Moves_Motors =
			new Ui::MenuBarAction(QString("&Motors"), wgts[WGT_MOTORS], signalDispatcher, menuBar);
	action_Absolute_Moves_Motors =
			new Ui::MenuBarAction(QString("&Motors"), wgts[WGT_MOTORS], signalDispatcher, menuBar);
	action_Joints = new Ui::MenuBarAction(QString("&Joints"), wgts[WGT_JOINTS], signalDispatcher, menuBar);
	action_Absolute_Moves_Xyz_Euler_Zyz =
			new Ui::MenuBarAction(QString("Xyz &Euler Zyz"), wgts[WGT_EULER], signalDispatcher, menuBar);
	action_Absolute_Moves_Xyz_Angle_Axis =
			new Ui::MenuBarAction(QString("Xyz &Angle Axis"), wgts[WGT_ANGLE_AXIS], signalDispatcher, menuBar);
	action_Xyz_Relative_Moves_Angle_Axis =
			new Ui::MenuBarAction(QString("Xyz &Angle Axis"), wgts[WGT_RELATIVE_ANGLE_AXIS], signalDispatcher, menuBar);
	action_Tool_Xyz_Euler_Zyz =
			new Ui::MenuBarAction(QString("Xyz &Euler Zyz"), wgts[WGT_TOOL_EULER], signalDispatcher, menuBar);
	action_Tool_Xyz_Angle_Axis =
			new Ui::MenuBarAction(QString("Xyz &Angle Axis"), wgts[WGT_TOOL_ANGLE_AXIS], signalDispatcher, menuBar);

	menu_Pre_Synchro_Moves = new QMenu(robot_menu);
	menu_Absolute_Moves = new QMenu(robot_menu);
	menu_Relative_Moves = new QMenu(robot_menu);
	menu_Tool = new QMenu(robot_menu);

	robot_menu->addAction(menu_Pre_Synchro_Moves->menuAction());
	robot_menu->addAction(menu_Absolute_Moves->menuAction());
	robot_menu->addAction(menu_Relative_Moves->menuAction());
	robot_menu->addAction(menu_Tool->menuAction());
	menu_Pre_Synchro_Moves->addAction(action_Synchronisation);
	menu_Pre_Synchro_Moves->addAction(action_Pre_Synchro_Moves_Motors);
	menu_Absolute_Moves->addAction(action_Absolute_Moves_Motors);
	menu_Absolute_Moves->addAction(action_Joints);
	menu_Absolute_Moves->addAction(action_Absolute_Moves_Xyz_Euler_Zyz);
	menu_Absolute_Moves->addAction(action_Absolute_Moves_Xyz_Angle_Axis);
	menu_Relative_Moves->addAction(action_Xyz_Relative_Moves_Angle_Axis);
	menu_Tool->addAction(action_Tool_Xyz_Euler_Zyz);
	menu_Tool->addAction(action_Tool_Xyz_Angle_Axis);

	robot_menu->setTitle(QApplication::translate("MainWindow", "Irp6&ot_m", 0, QApplication::UnicodeUTF8));
	menu_Pre_Synchro_Moves->setTitle(QApplication::translate("MainWindow", "P&re Synchro Moves", 0, QApplication::UnicodeUTF8));
	menu_Absolute_Moves->setTitle(QApplication::translate("MainWindow", "A&bsolute moves", 0, QApplication::UnicodeUTF8));
	menu_Relative_Moves->setTitle(QApplication::translate("MainWindow", "Re&lative Moves", 0, QApplication::UnicodeUTF8));
	menu_Tool->setTitle(QApplication::translate("MainWindow", "&Tool", 0, QApplication::UnicodeUTF8));

	menu_Preset_Positions = new QMenu(robot_menu);
	robot_menu->addSeparator();
	//robot_menu->addAction(action_Synchronisation);
	robot_menu->addAction(menu_Preset_Positions->menuAction());
	menu_Preset_Positions->addAction(action_Synchro_Position);
	menu_Preset_Positions->addAction(action_Front_Position);
	menu_Preset_Positions->addAction(action_Position_0);
	menu_Preset_Positions->addAction(action_Position_1);
	menu_Preset_Positions->addAction(action_Position_2);

	menu_Preset_Positions->setTitle(QApplication::translate("MainWindow", "&Preset positions", 0, QApplication::UnicodeUTF8));

	irp6_m::UiRobot::make_connections();

}

}
} //namespace ui
} //namespace mrrocpp
