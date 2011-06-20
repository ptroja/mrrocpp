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

namespace mrrocpp {
namespace ui {
namespace irp6p_m {
const std::string WGT_IRP6P_M_JOINTS = "WGT_IRP6P_M_JOINTS";
const std::string WGT_IRP6P_M_MOTORS = "WGT_IRP6P_M_MOTORS";
const std::string WGT_IRP6P_M_ANGLE_AXIS = "WGT_IRP6P_M_ANGLE_AXIS";
const std::string WGT_IRP6P_M_EULER = "WGT_IRP6P_M_EULER";
const std::string WGT_IRP6P_M_RELATIVE_ANGLE_AXIS = "WGT_IRP6P_M_RELATIVE_ANGLE_AXIS";
const std::string WGT_IRP6P_M_TOOL_ANGLE_AXIS = "WGT_IRP6P_M_TOOL_ANGLE_AXIS";
const std::string WGT_IRP6P_M_TOOL_EULER = "WGT_IRP6P_M_TOOL_EULER";
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

int UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::common::EcpRobot(*this);
	return 1;
}

int UiRobot::edp_create_int_extra_operations()
{
	wgt_motors->synchro_depended_init();
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
//	wgt_base *wgt;
//	wgt = wgt_joints");
//		wgt = new wgt_irp6_m_joints("Irp6p_m joints", interface, *this, interface.get_main_window());
//	wgt = wgt_motors");
//		wgt	= new wgt_irp6_m_motors("Irp6p_m motors", interface, *this, interface.get_main_window());
//	wgt = wgt_angle_axis");
//		wgt	= new wgt_irp6_m_angle_axis("Irp6p_m angle axis", interface, *this, interface.get_main_window());
//	wgt = wgt_euler");
//		wgt	 = new wgt_irp6_m_euler("Irp6p_m euler", interface, *this, interface.get_main_window());
//	wgt = wgt_relative_angle_axis");
//		wgt	= new wgt_irp6_m_relative_angle_axis("Irp6p_m relative angle axis", interface, *this, interface.get_main_window());
//	wgt = wgt_tool_angle_axis");
//		wgt	= new wgt_irp6_m_tool_angle_axis("Irp6p_m tool angle axis", interface, *this, interface.get_main_window());
//	wgt = wgt_tool_euler");
//		wgt	= new wgt_irp6_m_tool_euler("Irp6p_m tool euler", interface, *this, interface.get_main_window());



	wgt_joints=
		new wgt_irp6_m_joints("Irp6p_m joints", interface, *this, interface.get_main_window());
	wgt_motors=
		new wgt_irp6_m_motors("Irp6p_m motors", interface, *this, interface.get_main_window());
	wgt_angle_axis=
		new wgt_irp6_m_angle_axis("Irp6p_m angle axis", interface, *this, interface.get_main_window());
	wgt_euler=
		new wgt_irp6_m_euler("Irp6p_m euler", interface, *this, interface.get_main_window());
	wgt_relative_angle_axis=
		new wgt_irp6_m_relative_angle_axis("Irp6p_m relative angle axis", interface, *this, interface.get_main_window());
	wgt_tool_angle_axis=
		new wgt_irp6_m_tool_angle_axis("Irp6p_m tool angle axis", interface, *this, interface.get_main_window());
	wgt_tool_euler=
		new wgt_irp6_m_tool_euler("Irp6p_m tool euler", interface, *this, interface.get_main_window());

//	wndbase_m[WGT_IRP6P_M_JOINTS] = wgt_joints")->dwgt;
//	wndbase_m[WGT_IRP6P_M_MOTORS] = wgt_motors")->dwgt;
//	wndbase_m[WGT_IRP6P_M_ANGLE_AXIS] = wgt_angle_axis")->dwgt;
//	wndbase_m[WGT_IRP6P_M_EULER] = wgt_euler")->dwgt;
//	wndbase_m[WGT_IRP6P_M_RELATIVE_ANGLE_AXIS] = wgt_relative_angle_axis")->dwgt;
//	wndbase_m[WGT_IRP6P_M_TOOL_ANGLE_AXIS] = wgt_tool_angle_axis")->dwgt;
//	wndbase_m[WGT_IRP6P_M_TOOL_EULER] = wgt_tool_euler")->dwgt;

		wndbase_m[WGT_IRP6P_M_JOINTS] = wgt_joints->dwgt;
		wndbase_m[WGT_IRP6P_M_MOTORS] = wgt_motors->dwgt;
		wndbase_m[WGT_IRP6P_M_ANGLE_AXIS] = wgt_angle_axis->dwgt;
		wndbase_m[WGT_IRP6P_M_EULER] = wgt_euler->dwgt;
		wndbase_m[WGT_IRP6P_M_RELATIVE_ANGLE_AXIS] = wgt_relative_angle_axis->dwgt;
		wndbase_m[WGT_IRP6P_M_TOOL_ANGLE_AXIS] = wgt_tool_angle_axis->dwgt;
		wndbase_m[WGT_IRP6P_M_TOOL_EULER] = wgt_tool_euler->dwgt;
}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();

	switch (state.edp.state)
	{

		case -1:
			mw->enable_menu_item(false, 1, robot_menu);
			break;
		case 0:
			mw->enable_menu_item(false, 5, menuirp6p_m_Pre_Synchro_Moves, menuirp6p_m_Absolute_Moves, menuirp6p_m_Preset_Positions, menuirp6p_m_Relative_Moves, menuirp6p_m_Tool);
			mw->enable_menu_item(false, 1, EDP_Unload);
			mw->enable_menu_item(true, 1, robot_menu);
			mw->enable_menu_item(true, 1, EDP_Load);
			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, robot_menu);
			mw->enable_menu_item(true, 1, mw->getMenuBar()->actionall_EDP_Unload);

			//ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, menuirp6p_m_Pre_Synchro_Moves);
				mw->enable_menu_item(true, 1, mw->getMenuBar()->menuall_Preset_Positions);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 4, menuirp6p_m_Absolute_Moves, menuirp6p_m_Preset_Positions, menuirp6p_m_Relative_Moves, menuirp6p_m_Tool);
						mw->enable_menu_item(true, 1, EDP_Unload);
						mw->enable_menu_item(false, 1, EDP_Load);
						block_ecp_trigger();
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 4, menuirp6p_m_Absolute_Moves, menuirp6p_m_Preset_Positions, menuirp6p_m_Relative_Moves, menuirp6p_m_Tool);
						mw->enable_menu_item(false, 2, EDP_Unload, EDP_Load);
						block_ecp_trigger();
						break;
					case common::UI_MP_TASK_RUNNING:
						unblock_ecp_trigger();
						break;
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 4, menuirp6p_m_Absolute_Moves, menuirp6p_m_Preset_Positions, menuirp6p_m_Relative_Moves, menuirp6p_m_Tool);
						block_ecp_trigger();
						break;
					default:
						break;
				}

			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, menuirp6p_m_Pre_Synchro_Moves);
				mw->enable_menu_item(true, 1, EDP_Unload);
				mw->enable_menu_item(false, 1, EDP_Load);

			}
			break;
		default:
			break;

	}

	return 1;

}

// aktualizacja ustawien przyciskow
int UiRobot::process_control_window_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{

	if (state.edp.state <= 0) {// edp wylaczone
		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_trigger);
		 */
	} else if (state.edp.state == 1) {// edp wlaczone reader czeka na start
		wlacz_PtButton_wnd_processes_control_all_reader_start = true;
		/* TR
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_trigger);
		 */
	} else if (state.edp.state == 2) {// edp wlaczone reader czeka na stop
		wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
		wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;
		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_start);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_stop);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_trigger);
		 */
	}

	state.edp.last_state = state.edp.state;
	return 0;
}

void UiRobot::make_connections()
{
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	connect(actionirp6p_m_Synchronisation, 				SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), 				Qt::AutoCompatConnection);
	connect(actionirp6p_m_Pre_Synchro_Moves_Motors, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Pre_Synchro_Moves_Motors_triggered(mrrocpp::ui::common::UiRobot*)),		Qt::AutoCompatConnection);
	connect(actionirp6p_m_Absolute_Moves_Motors, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Absolute_Moves_Motors_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionirp6p_m_Joints, 						SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Joints_triggered(mrrocpp::ui::common::UiRobot*)), 						Qt::AutoCompatConnection);
	connect(actionirp6p_m_Absolute_Moves_Xyz_Euler_Zyz,	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Absolute_Moves_Xyz_Euler_Zyz_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionirp6p_m_Absolute_Moves_Xyz_Angle_Axis,SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Absolute_Moves_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot*)),Qt::AutoCompatConnection);
	connect(actionirp6p_m_Xyz_Relative_Moves_Angle_Axis,SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Relative_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot*)),		Qt::AutoCompatConnection);
	connect(actionirp6p_m_Synchro_Position, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)),				Qt::AutoCompatConnection);
	connect(actionirp6p_m_Front_Position, 				SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Front_Position_triggered(mrrocpp::ui::common::UiRobot*)), 				Qt::AutoCompatConnection);
	connect(actionirp6p_m_Position_0, 					SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
	connect(actionirp6p_m_Position_1, 					SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
	connect(actionirp6p_m_Position_2, 					SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
	connect(actionirp6p_m_Tool_Xyz_Euler_Zyz, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Tool_Xyz_Euler_Zyz_triggered(mrrocpp::ui::common::UiRobot*)), 			Qt::AutoCompatConnection);
	connect(actionirp6p_m_Tool_Xyz_Angle_Axis, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Tool_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot*)), 			Qt::AutoCompatConnection);
}


void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

    actionirp6p_m_Synchronisation 				= new Ui::MenuBarAction(QString("&Synchronisation"),this, menuBar);
    actionirp6p_m_Synchro_Position 				= new Ui::MenuBarAction(QString("&Synchro Position"),this, menuBar);
    actionirp6p_m_Front_Position 				= new Ui::MenuBarAction(QString("&Front Position"),this, menuBar);
    actionirp6p_m_Position_0 					= new Ui::MenuBarAction(QString("Position &0"),this, menuBar);
    actionirp6p_m_Position_1 					= new Ui::MenuBarAction(QString("Position &1"),this, menuBar);
    actionirp6p_m_Position_2 					= new Ui::MenuBarAction(QString("Position &2"),this, menuBar);
    actionirp6p_m_Pre_Synchro_Moves_Motors 		= new Ui::MenuBarAction(QString("&Motors"),this, menuBar);
	actionirp6p_m_Absolute_Moves_Motors 		= new Ui::MenuBarAction(QString("&Motors"),this, menuBar);
	actionirp6p_m_Joints 						= new Ui::MenuBarAction(QString("&Joints"),this, menuBar);
	actionirp6p_m_Absolute_Moves_Xyz_Euler_Zyz 	= new Ui::MenuBarAction(QString("Xyz &Euler Zyz"),this, menuBar);
	actionirp6p_m_Absolute_Moves_Xyz_Angle_Axis = new Ui::MenuBarAction(QString("Xyz &Angle Axis"),this, menuBar);
	actionirp6p_m_Tool_Xyz_Euler_Zyz 			= new Ui::MenuBarAction(QString("Xyz &Euler Zyz"),this, menuBar);
	actionirp6p_m_Tool_Xyz_Angle_Axis 			= new Ui::MenuBarAction(QString("Xyz &Angle Axis"),this, menuBar);
	actionirp6p_m_Motors						= new Ui::MenuBarAction(QString("&Motors"),this, menuBar);
	actionirp6p_m_Xyz_Relative_Moves_Angle_Axis = new Ui::MenuBarAction(QString("Xyz &Angle Axis"),this, menuBar);

	menuirp6p_m_Pre_Synchro_Moves = new QMenu(robot_menu);
	menuirp6p_m_Preset_Positions = new QMenu(robot_menu);
	menuirp6p_m_Absolute_Moves = new QMenu(robot_menu);
	menuirp6p_m_Relative_Moves = new QMenu(robot_menu);
	menuirp6p_m_Tool = new QMenu(robot_menu);

	robot_menu->addSeparator();
	robot_menu->addAction(menuirp6p_m_Pre_Synchro_Moves->menuAction());
	robot_menu->addAction(menuirp6p_m_Absolute_Moves->menuAction());
	robot_menu->addAction(menuirp6p_m_Relative_Moves->menuAction());
	robot_menu->addAction(menuirp6p_m_Preset_Positions->menuAction());
	robot_menu->addSeparator();
	robot_menu->addAction(menuirp6p_m_Tool->menuAction());
	menuirp6p_m_Pre_Synchro_Moves->addAction(actionirp6p_m_Synchronisation);
	menuirp6p_m_Pre_Synchro_Moves->addAction(actionirp6p_m_Pre_Synchro_Moves_Motors);
	menuirp6p_m_Preset_Positions->addAction(actionirp6p_m_Synchro_Position);
	menuirp6p_m_Preset_Positions->addAction(actionirp6p_m_Front_Position);
	menuirp6p_m_Preset_Positions->addAction(actionirp6p_m_Position_0);
	menuirp6p_m_Preset_Positions->addAction(actionirp6p_m_Position_1);
	menuirp6p_m_Preset_Positions->addAction(actionirp6p_m_Position_2);
	menuirp6p_m_Absolute_Moves->addAction(actionirp6p_m_Absolute_Moves_Motors);
	menuirp6p_m_Absolute_Moves->addAction(actionirp6p_m_Joints);
	menuirp6p_m_Absolute_Moves->addAction(actionirp6p_m_Absolute_Moves_Xyz_Euler_Zyz);
	menuirp6p_m_Absolute_Moves->addAction(actionirp6p_m_Absolute_Moves_Xyz_Angle_Axis);
	menuirp6p_m_Relative_Moves->addAction(actionirp6p_m_Xyz_Relative_Moves_Angle_Axis);
	menuirp6p_m_Tool->addAction(actionirp6p_m_Tool_Xyz_Euler_Zyz);
	menuirp6p_m_Tool->addAction(actionirp6p_m_Tool_Xyz_Angle_Axis);

    robot_menu->setTitle(QApplication::translate("MainWindow", "Irp6&p_m", 0, QApplication::UnicodeUTF8));
    menuirp6p_m_Pre_Synchro_Moves->setTitle(QApplication::translate("MainWindow", "P&re Synchro Moves", 0, QApplication::UnicodeUTF8));
    menuirp6p_m_Preset_Positions->setTitle(QApplication::translate("MainWindow", "Pr&eset Positions", 0, QApplication::UnicodeUTF8));
    menuirp6p_m_Absolute_Moves->setTitle(QApplication::translate("MainWindow", "A&bsolute moves", 0, QApplication::UnicodeUTF8));
    menuirp6p_m_Relative_Moves->setTitle(QApplication::translate("MainWindow", "Re&lative moves", 0, QApplication::UnicodeUTF8));
    menuirp6p_m_Tool->setTitle(QApplication::translate("MainWindow", "&Tool", 0, QApplication::UnicodeUTF8));

    make_connections();
}



}
} //namespace ui
} //namespace mrrocpp
