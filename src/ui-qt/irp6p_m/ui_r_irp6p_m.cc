/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "../irp6_m/wgt_irp6_m_joints.h"
#include "../irp6_m/wgt_irp6_m_motors.h"
#include "../irp6_m/wgt_irp6_m_euler.h"
#include "../irp6_m/wgt_irp6_m_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_relative_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_euler.h"

#include "ui_r_irp6p_m.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"
#include "../base/ui_robot.h"
#include <boost/foreach.hpp>

namespace mrrocpp {
namespace ui {
namespace irp6p_m {
//const std::string WGT_IRP6P_M_JOINTS = "WGT_IRP6P_M_JOINTS";
//const std::string WGT_IRP6P_M_MOTORS = "WGT_IRP6P_M_MOTORS";
//const std::string WGT_IRP6P_M_ANGLE_AXIS = "WGT_IRP6P_M_ANGLE_AXIS";
//const std::string WGT_IRP6P_M_EULER = "WGT_IRP6P_M_EULER";
//const std::string WGT_IRP6P_M_RELATIVE_ANGLE_AXIS = "WGT_IRP6P_M_RELATIVE_ANGLE_AXIS";
//const std::string WGT_IRP6P_M_TOOL_ANGLE_AXIS = "WGT_IRP6P_M_TOOL_ANGLE_AXIS";
//const std::string WGT_IRP6P_M_TOOL_EULER = "WGT_IRP6P_M_TOOL_EULER";
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
//	return 1;
}

int UiRobot::edp_create_int_extra_operations()
{
	wgts[WGT_MOTORS]->synchro_depended_init();
	return 1;
}

int UiRobot::move_to_synchro_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = 0.0;
	}
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::execute_motor_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_front_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.front_position[i];
	}
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.preset_position[variant][i];
	}
	eb.command(boost::bind(&ui::irp6p_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::irp6p_m::UiRobot::synchronise_int, &(*this)));

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
		irp6_m::UiRobot(_interface, lib::irp6p_m::ROBOT_NAME, lib::irp6p_m::NUM_OF_SERVOS)
{
	add_wgt <wgt_irp6_m_joints>(WGT_JOINTS, "Irp6p_m joints");
	add_wgt <wgt_irp6_m_motors>(WGT_MOTORS, "Irp6p_m motors");
	add_wgt <wgt_irp6_m_angle_axis>(WGT_ANGLE_AXIS, "Irp6p_m angle axis");
	add_wgt <wgt_irp6_m_euler>(WGT_EULER, "Irp6p_m euler");
	add_wgt <wgt_irp6_m_relative_angle_axis>(WGT_RELATIVE_ANGLE_AXIS, "Irp6p_m relative angle axis");
	add_wgt <wgt_irp6_m_tool_angle_axis>(WGT_TOOL_ANGLE_AXIS, "Irp6p_m tool angle axis");
	add_wgt <wgt_irp6_m_tool_euler>(WGT_TOOL_EULER, "Irp6p_m tool euler");
}

int UiRobot::manage_interface()
{
//	MainWindow *mw = interface.get_main_window();
	irp6_m::UiRobot::manage_interface();

	return 1;
}

// aktualizacja ustawien przyciskow
int UiRobot::process_control_window_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{

	if (state.edp.state <= 0) { // edp wylaczone

	} else if (state.edp.state == 1) { // edp wlaczone reader czeka na start
		wlacz_PtButton_wnd_processes_control_all_reader_start = true;

	} else if (state.edp.state == 2) { // edp wlaczone reader czeka na stop
		wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
		wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;

	}

	state.edp.last_state = state.edp.state;
	return 0;
}

void UiRobot::make_connections()
{
//	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

//	connect(actionirp6p_m_Synchronisation, 				SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), 				Qt::AutoCompatConnection);
////	connect(actionirp6p_m_Pre_Synchro_Moves_Motors, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Pre_Synchro_Moves_Motors_triggered(mrrocpp::ui::common::UiRobot*)),		Qt::AutoCompatConnection);
////	connect(actionirp6p_m_Absolute_Moves_Motors, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Absolute_Moves_Motors_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
////	connect(actionirp6p_m_Joints, 						SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Joints_triggered(mrrocpp::ui::common::UiRobot*)), 						Qt::AutoCompatConnection);
////	connect(actionirp6p_m_Absolute_Moves_Xyz_Euler_Zyz,	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Absolute_Moves_Xyz_Euler_Zyz_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
////	connect(actionirp6p_m_Absolute_Moves_Xyz_Angle_Axis,SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Absolute_Moves_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot*)),Qt::AutoCompatConnection);
////	connect(actionirp6p_m_Xyz_Relative_Moves_Angle_Axis,SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Relative_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot*)),		Qt::AutoCompatConnection);
//	connect(actionirp6p_m_Synchro_Position, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)),				Qt::AutoCompatConnection);
//	connect(actionirp6p_m_Front_Position, 				SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Front_Position_triggered(mrrocpp::ui::common::UiRobot*)), 				Qt::AutoCompatConnection);
//	connect(actionirp6p_m_Position_0, 					SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
//	connect(actionirp6p_m_Position_1, 					SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
//	connect(actionirp6p_m_Position_2, 					SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
////	connect(actionirp6p_m_Tool_Xyz_Euler_Zyz, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Tool_Xyz_Euler_Zyz_triggered(mrrocpp::ui::common::UiRobot*)), 			Qt::AutoCompatConnection);
//	connect(actionirp6p_m_Tool_Xyz_Angle_Axis, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Tool_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot*)), 			Qt::AutoCompatConnection);
}

void UiRobot::setup_menubar()
{
	irp6_m::UiRobot::setup_menubar();
//	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
//	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();
//
//    actionirp6p_m_Pre_Synchro_Moves_Motors 		= new Ui::MenuBarAction(QString("&Motors"), wgts[WGT_MOTORS], signalDispatcher, menuBar);
//	actionirp6p_m_Absolute_Moves_Motors 		= new Ui::MenuBarAction(QString("&Motors"), wgts[WGT_MOTORS], signalDispatcher, menuBar);
//	actionirp6p_m_Joints 						= new Ui::MenuBarAction(QString("&Joints"), wgts[WGT_JOINTS], signalDispatcher, menuBar);
//	actionirp6p_m_Absolute_Moves_Xyz_Euler_Zyz 	= new Ui::MenuBarAction(QString("Xyz &Euler Zyz"),wgts[WGT_EULER], signalDispatcher, menuBar);
//	actionirp6p_m_Absolute_Moves_Xyz_Angle_Axis = new Ui::MenuBarAction(QString("Xyz &Angle Axis"), wgts[WGT_ANGLE_AXIS], signalDispatcher, menuBar);
//	actionirp6p_m_Tool_Xyz_Euler_Zyz 			= new Ui::MenuBarAction(QString("Xyz &Euler Zyz"), wgts[WGT_TOOL_EULER], signalDispatcher, menuBar);
//	actionirp6p_m_Tool_Xyz_Angle_Axis 			= new Ui::MenuBarAction(QString("Xyz &Angle Axis"), wgts[WGT_TOOL_ANGLE_AXIS], signalDispatcher, menuBar);
//	actionirp6p_m_Motors						= new Ui::MenuBarAction(QString("&Motors"), wgts[WGT_MOTORS], signalDispatcher, menuBar);
//	actionirp6p_m_Xyz_Relative_Moves_Angle_Axis = new Ui::MenuBarAction(QString("Xyz &Angle Axis"), wgts[WGT_RELATIVE_ANGLE_AXIS], signalDispatcher, menuBar);
//
//	menuirp6p_m_Pre_Synchro_Moves = new QMenu(robot_menu);
//	menuirp6p_m_Absolute_Moves = new QMenu(robot_menu);
//	menuirp6p_m_Relative_Moves = new QMenu(robot_menu);
//	menuirp6p_m_Tool = new QMenu(robot_menu);
//
//
//
//
//
//
//	robot_menu->addSeparator();
//	robot_menu->addAction(menuirp6p_m_Pre_Synchro_Moves->menuAction());
//	robot_menu->addAction(menuirp6p_m_Absolute_Moves->menuAction());
//	robot_menu->addAction(menuirp6p_m_Relative_Moves->menuAction());
//	robot_menu->addSeparator();
//	robot_menu->addAction(menuirp6p_m_Tool->menuAction());
//	menuirp6p_m_Pre_Synchro_Moves->addAction(actionirp6p_m_Pre_Synchro_Moves_Motors);
//	menuirp6p_m_Absolute_Moves->addAction(actionirp6p_m_Absolute_Moves_Motors);
//	menuirp6p_m_Absolute_Moves->addAction(actionirp6p_m_Joints);
//	menuirp6p_m_Absolute_Moves->addAction(actionirp6p_m_Absolute_Moves_Xyz_Euler_Zyz);
//	menuirp6p_m_Absolute_Moves->addAction(actionirp6p_m_Absolute_Moves_Xyz_Angle_Axis);
//	menuirp6p_m_Relative_Moves->addAction(actionirp6p_m_Xyz_Relative_Moves_Angle_Axis);
//	menuirp6p_m_Tool->addAction(actionirp6p_m_Tool_Xyz_Euler_Zyz);
//	menuirp6p_m_Tool->addAction(actionirp6p_m_Tool_Xyz_Angle_Axis);

	robot_menu->setTitle(QApplication::translate("MainWindow", "Irp6&p_m", 0, QApplication::UnicodeUTF8));
//    menuirp6p_m_Pre_Synchro_Moves->setTitle(QApplication::translate("MainWindow", "P&re Synchro Moves", 0, QApplication::UnicodeUTF8));
//    menuirp6p_m_Absolute_Moves->setTitle(QApplication::translate("MainWindow", "A&bsolute moves", 0, QApplication::UnicodeUTF8));
//    menuirp6p_m_Relative_Moves->setTitle(QApplication::translate("MainWindow", "Re&lative moves", 0, QApplication::UnicodeUTF8));
//    menuirp6p_m_Tool->setTitle(QApplication::translate("MainWindow", "&Tool", 0, QApplication::UnicodeUTF8));

	make_connections();
}

}
} //namespace ui
} //namespace mrrocpp
