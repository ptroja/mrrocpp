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

	common::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case common::UI_EDP_INACTIVE:

			break;
		case common::UI_EDP_OFF:
			menu_Preset_Positions->setEnabled(false);
			action_Synchronisation->setEnabled(false);
			menu_Pre_Synchro_Moves->setEnabled(false);
			menu_Absolute_Moves->setEnabled(false);
			menu_Relative_Moves->setEnabled(false);
			menu_Tool->setEnabled(false);

			break;
		case common::UI_EDP_WAITING_TO_START_READER:
		case common::UI_EDP_WAITING_TO_STOP_READER:
			menu_Pre_Synchro_Moves->setEnabled(false);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				action_Synchronisation->setEnabled(false);

				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
					case common::UI_MP_WAITING_FOR_START_PULSE:
						menu_Preset_Positions->setEnabled(true);
						menu_Absolute_Moves->setEnabled(true);
						menu_Relative_Moves->setEnabled(true);
						menu_Tool->setEnabled(true);

						break;
					case common::UI_MP_TASK_RUNNING:

						break;
					case common::UI_MP_TASK_PAUSED:
						menu_Preset_Positions->setEnabled(false);
						menu_Absolute_Moves->setEnabled(false);
						menu_Relative_Moves->setEnabled(false);
						menu_Tool->setEnabled(false);
						break;
					default:
						break;
				}

			} else // jesli robot jest niezsynchronizowany
			{
				action_Synchronisation->setEnabled(true);
				menu_Pre_Synchro_Moves->setEnabled(true);
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

	// connections
	connect(action_Synchronisation, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Synchro_Position, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Front_Position, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Front_Position_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_0, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_1, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_2, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);

}

}
} //namespace ui
} //namespace mrrocpp
