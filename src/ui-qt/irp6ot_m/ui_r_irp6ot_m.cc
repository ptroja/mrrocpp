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

#include "ui_r_irp6ot_m.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"
#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"


#include "../base/wgt_process_control.h"

namespace mrrocpp {
namespace ui {
namespace irp6ot_m {
const std::string WGT_IRP6OT_M_JOINTS = "WGT_IRP6OT_M_JOINTS";
const std::string WGT_IRP6OT_M_MOTORS = "WGT_IRP6OT_M_MOTORS";
const std::string WGT_IRP6OT_M_ANGLE_AXIS = "WGT_IRP6OT_M_ANGLE_AXIS";
const std::string WGT_IRP6OT_M_EULER = "WGT_IRP6OT_M_EULER";
const std::string WGT_IRP6OT_M_RELATIVE_ANGLE_AXIS = "WGT_IRP6OT_M_RELATIVE_ANGLE_AXIS";
const std::string WGT_IRP6OT_M_TOOL_ANGLE_AXIS = "WGT_IRP6OT_M_TOOL_ANGLE_AXIS";
const std::string WGT_IRP6OT_M_TOOL_EULER = "WGT_IRP6OT_M_TOOL_EULER";
//
//
// KLASA UiRobot
//
//


//void UiRobot::on_actionirp6ot_m_EDP_Load_triggered()
//{
//	interface.get_main_window()->ui_robot_action(this, &UiRobot::edp_create);
//}



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
	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_motor_motion, &(*this)));
	interface.print_on_sr("juhuuu, jesteśmy w irp6ot synchronizacji");

	return 1;
}

int UiRobot::move_to_front_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.front_position[i];
	}

	//	printf(" move_to_front_position state.edp.front_position: %lf, %lf, %lf, %lf, %lf, %lf\n ", state.edp.front_position[0], state.edp.front_position[1], state.edp.front_position[2], state.edp.front_position[3], state.edp.front_position[4], state.edp.front_position[5]);

	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.preset_position[variant][i];
	}
	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

UiRobot::UiRobot(common::Interface& _interface) :
	irp6_m::UiRobot(_interface, lib::irp6ot_m::ROBOT_NAME, lib::irp6ot_m::NUM_OF_SERVOS)
{
//	wgt_base *wgt;
//	wgt = getWgtByName("wgt_joints");
//		wgt = new wgt_irp6_m_joints("Irp6ot_m joints", interface, *this, interface.get_main_window());
//	wgt = getWgtByName("wgt_motors");
//		=new wgt_irp6_m_motors("Irp6ot_m motors", interface, *this, interface.get_main_window());
//	wgt = getWgtByName("wgt_angle_axis");
//		wgt	= new wgt_irp6_m_angle_axis("Irp6ot_m angle axis", interface, *this, interface.get_main_window());
//	wgt = getWgtByName("wgt_euler");
//		wgt	= new wgt_irp6_m_euler("Irp6ot_m euler", interface, *this, interface.get_main_window());
//	wgt = getWgtByName("wgt_relative_angle_axis");
//		wgt	= new wgt_irp6_m_relative_angle_axis("Irp6ot_m relative angle axis", interface, *this, interface.get_main_window());
//	wgt = getWgtByName("wgt_tool_angle_axis");
//		wgt	= new wgt_irp6_m_tool_angle_axis("Irp6ot_m tool angle axis", interface, *this, interface.get_main_window());
//	wgt = getWgtByName("wgt_tool_euler");
//		wgt	= new wgt_irp6_m_tool_euler("Irp6ot_m tool euler", interface, *this, interface.get_main_window());

	//wgt_base *wgt;
		wgt_joints=
			new wgt_irp6_m_joints("Irp6ot_m joints", interface, *this, interface.get_main_window());
		wgt_motors=
			new wgt_irp6_m_motors("Irp6ot_m motors", interface, *this, interface.get_main_window());
		wgt_angle_axis=
			new wgt_irp6_m_angle_axis("Irp6ot_m angle axis", interface, *this, interface.get_main_window());
		wgt_euler=
			new wgt_irp6_m_euler("Irp6ot_m euler", interface, *this, interface.get_main_window());
		wgt_relative_angle_axis=
			new wgt_irp6_m_relative_angle_axis("Irp6ot_m relative angle axis", interface, *this, interface.get_main_window());
		wgt_tool_angle_axis=
			new wgt_irp6_m_tool_angle_axis("Irp6ot_m tool angle axis", interface, *this, interface.get_main_window());
		wgt_tool_euler=
			new wgt_irp6_m_tool_euler("Irp6ot_m tool euler", interface, *this, interface.get_main_window());


//	wndbase_m[WGT_IRP6OT_M_JOINTS] = getWgtByName("wgt_joints")->dwgt;
//	wndbase_m[WGT_IRP6OT_M_MOTORS] = getWgtByName("wgt_motors")->dwgt;
//	wndbase_m[WGT_IRP6OT_M_ANGLE_AXIS] = getWgtByName("wgt_angle_axis")->dwgt;
//	wndbase_m[WGT_IRP6OT_M_EULER] = getWgtByName("wgt_euler")->dwgt;
//	wndbase_m[WGT_IRP6OT_M_RELATIVE_ANGLE_AXIS] = getWgtByName("wgt_relative_angle_axis")->dwgt;
//	wndbase_m[WGT_IRP6OT_M_TOOL_ANGLE_AXIS] = getWgtByName("wgt_tool_angle_axis")->dwgt;
//	wndbase_m[WGT_IRP6OT_M_TOOL_EULER] = getWgtByName("wgt_tool_euler")->dwgt;

	wndbase_m[WGT_IRP6OT_M_JOINTS] = wgt_joints->dwgt;
	wndbase_m[WGT_IRP6OT_M_MOTORS] = wgt_motors->dwgt;
	wndbase_m[WGT_IRP6OT_M_ANGLE_AXIS] = wgt_angle_axis->dwgt;
	wndbase_m[WGT_IRP6OT_M_EULER] = wgt_euler->dwgt;
	wndbase_m[WGT_IRP6OT_M_RELATIVE_ANGLE_AXIS] = wgt_relative_angle_axis->dwgt;
	wndbase_m[WGT_IRP6OT_M_TOOL_ANGLE_AXIS] = wgt_tool_angle_axis->dwgt;
	wndbase_m[WGT_IRP6OT_M_TOOL_EULER] = wgt_tool_euler->dwgt;

}

void UiRobot::make_connections()
{
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	connect(actionirp6ot_m_Synchronisation, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), 				Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Pre_Synchro_Moves_Motors, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Pre_Synchro_Moves_Motors_triggered(mrrocpp::ui::common::UiRobot*)),		Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Absolute_Moves_Motors, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Absolute_Moves_Motors_triggered(mrrocpp::ui::common::UiRobot*)), 		Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Joints, 						SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Joints_triggered(mrrocpp::ui::common::UiRobot*)), 						Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz,SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Absolute_Moves_Xyz_Euler_Zyz_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis,SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Absolute_Moves_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot*)),Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Relative_Xyz_Angle_Axis, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Relative_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot*)),		Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Synchro_Position, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)),				Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Front_Position, 				SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Front_Position_triggered(mrrocpp::ui::common::UiRobot*)), 				Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Position_0, 					SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Position_1, 					SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Position_2, 					SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Tool_Xyz_Euler_Zyz, 			SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Tool_Xyz_Euler_Zyz_triggered(mrrocpp::ui::common::UiRobot*)), 			Qt::AutoCompatConnection);
	connect(actionirp6ot_m_Tool_Xyz_Angle_Axis, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Tool_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot*)), 			Qt::AutoCompatConnection);

}


void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

    actionirp6ot_m_Synchronisation 				= new Ui::MenuBarAction(QString("&Synchronisation"),this, menuBar);
    actionirp6ot_m_Synchro_Position 			= new Ui::MenuBarAction(QString("&Synchro Position"),this, menuBar);
    actionirp6ot_m_Front_Position 				= new Ui::MenuBarAction(QString("&Front Position"),this, menuBar);
    actionirp6ot_m_Position_0 					= new Ui::MenuBarAction(QString("Position &0"),this, menuBar);
    actionirp6ot_m_Position_1 					= new Ui::MenuBarAction(QString("Position &1"),this, menuBar);
    actionirp6ot_m_Position_2 					= new Ui::MenuBarAction(QString("Position &2"),this, menuBar);
    actionirp6ot_m_Pre_Synchro_Moves_Motors 	= new Ui::MenuBarAction(QString("&Motors"),this, menuBar);
	actionirp6ot_m_Absolute_Moves_Motors 		= new Ui::MenuBarAction(QString("&Motors"),this, menuBar);
	actionirp6ot_m_Joints 						= new Ui::MenuBarAction(QString("&Joints"),this, menuBar);
	actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz = new Ui::MenuBarAction(QString("Xyz &Euler Zyz"),this, menuBar);
	actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis = new Ui::MenuBarAction(QString("Xyz &Angle Axis"),this, menuBar);
	actionirp6ot_m_Relative_Xyz_Angle_Axis 		= new Ui::MenuBarAction(QString("Xyz &Angle Axis"),this, menuBar);
	actionirp6ot_m_Tool_Xyz_Euler_Zyz 			= new Ui::MenuBarAction(QString("Xyz &Euler Zyz"),this, menuBar);
	actionirp6ot_m_Tool_Xyz_Angle_Axis 			= new Ui::MenuBarAction(QString("Xyz &Angle Axis"),this, menuBar);


    menuirp6ot_m_Pre_Synchro_Moves = new QMenu(robot_menu);
    menuirp6ot_m_Preset_Positions = new QMenu(robot_menu);
    menuirp6ot_m_Absolute_moves = new QMenu(robot_menu);
    menuIrp6ot_m_Relative_Moves = new QMenu(robot_menu);
    menuirp6ot_m_Tool = new QMenu(robot_menu);


	robot_menu->addAction(menuirp6ot_m_Pre_Synchro_Moves->menuAction());
	robot_menu->addAction(menuirp6ot_m_Absolute_moves->menuAction());
	robot_menu->addAction(menuIrp6ot_m_Relative_Moves->menuAction());
	robot_menu->addAction(menuirp6ot_m_Preset_Positions->menuAction());
	robot_menu->addAction(menuirp6ot_m_Tool->menuAction());
	menuirp6ot_m_Pre_Synchro_Moves->addAction(actionirp6ot_m_Synchronisation);
	menuirp6ot_m_Pre_Synchro_Moves->addAction(actionirp6ot_m_Pre_Synchro_Moves_Motors);
	menuirp6ot_m_Preset_Positions->addAction(actionirp6ot_m_Synchro_Position);
	menuirp6ot_m_Preset_Positions->addAction(actionirp6ot_m_Front_Position);
	menuirp6ot_m_Preset_Positions->addAction(actionirp6ot_m_Position_0);
	menuirp6ot_m_Preset_Positions->addAction(actionirp6ot_m_Position_1);
	menuirp6ot_m_Preset_Positions->addAction(actionirp6ot_m_Position_2);
	menuirp6ot_m_Absolute_moves->addAction(actionirp6ot_m_Absolute_Moves_Motors);
	menuirp6ot_m_Absolute_moves->addAction(actionirp6ot_m_Joints);
	menuirp6ot_m_Absolute_moves->addAction(actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz);
	menuirp6ot_m_Absolute_moves->addAction(actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis);
	menuIrp6ot_m_Relative_Moves->addAction(actionirp6ot_m_Relative_Xyz_Angle_Axis);
	menuirp6ot_m_Tool->addAction(actionirp6ot_m_Tool_Xyz_Euler_Zyz);
	menuirp6ot_m_Tool->addAction(actionirp6ot_m_Tool_Xyz_Angle_Axis);

    robot_menu->setTitle(QApplication::translate("MainWindow", "Irp6&ot_m", 0, QApplication::UnicodeUTF8));
    menuirp6ot_m_Pre_Synchro_Moves->setTitle(QApplication::translate("MainWindow", "P&re Synchro Moves", 0, QApplication::UnicodeUTF8));
    menuirp6ot_m_Preset_Positions->setTitle(QApplication::translate("MainWindow", "&Preset Positions", 0, QApplication::UnicodeUTF8));
    menuirp6ot_m_Absolute_moves->setTitle(QApplication::translate("MainWindow", "A&bsolute moves", 0, QApplication::UnicodeUTF8));
    menuIrp6ot_m_Relative_Moves->setTitle(QApplication::translate("MainWindow", "Re&lative Moves", 0, QApplication::UnicodeUTF8));
    menuirp6ot_m_Tool->setTitle(QApplication::translate("MainWindow", "&Tool", 0, QApplication::UnicodeUTF8));

	make_connections();
}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::synchronise_int, &(*this)));

	return 1;

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

	switch (state.edp.state)
	{

		case -1:
			mw->enable_menu_item(false, 1, robot_menu);

			break;
		case 0:
			mw->enable_menu_item(false, 5, menuirp6ot_m_Pre_Synchro_Moves, menuirp6ot_m_Absolute_moves, menuIrp6ot_m_Relative_Moves, menuirp6ot_m_Tool, menuirp6ot_m_Preset_Positions);
			mw->enable_menu_item(false, 1, EDP_Unload);
			mw->enable_menu_item(true, 1, robot_menu);
			mw->enable_menu_item(true, 1, EDP_Load);

			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, robot_menu);
			mw->enable_menu_item(true, 1, menuBar->actionall_EDP_Unload);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, menuirp6ot_m_Pre_Synchro_Moves);
				mw->enable_menu_item(true, 1, menuBar->menuall_Preset_Positions);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 4, menuirp6ot_m_Absolute_moves, menuIrp6ot_m_Relative_Moves, menuirp6ot_m_Tool, menuirp6ot_m_Preset_Positions);
						mw->enable_menu_item(true, 1, EDP_Unload);
						mw->enable_menu_item(false, 1, EDP_Load);
						block_ecp_trigger();
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 4, menuirp6ot_m_Absolute_moves, menuIrp6ot_m_Relative_Moves, menuirp6ot_m_Tool, menuirp6ot_m_Preset_Positions);
						mw->enable_menu_item(false, 2, EDP_Load, EDP_Unload);
						block_ecp_trigger();
						break;
					case common::UI_MP_TASK_RUNNING:
						unblock_ecp_trigger();
						break;
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 4, menuirp6ot_m_Absolute_moves, menuIrp6ot_m_Relative_Moves, menuirp6ot_m_Tool, menuirp6ot_m_Preset_Positions);
						block_ecp_trigger();
						break;
					default:
						break;
				}

			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, menuirp6ot_m_Pre_Synchro_Moves);
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
	//wgt_process_control *ui = interface.get_process_control_window(); //TODO: zmienic sposob

	if (state.edp.state <= 0) {// edp wylaczone
//		ui->all_reader_start_pushButton->setDisabled(true);
//		ui->all_reader_stop_pushButton->setDisabled(true);
//		ui->all_reader_trigger_pushButton->setDisabled(true);

		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		 */
	} else if (state.edp.state == 1) {// edp wlaczone reader czeka na start
		wlacz_PtButton_wnd_processes_control_all_reader_start = true;
//		ui->all_reader_start_pushButton->setEnabled(true);
//		ui->all_reader_stop_pushButton->setDisabled(true);
//		ui->all_reader_trigger_pushButton->setDisabled(true);
		/* TR
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		 */
	} else if (state.edp.state == 2) {// edp wlaczone reader czeka na stop
		wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
		wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;
		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		 */
	}

	state.edp.last_state = state.edp.state;

	return 1;

}

}
} //namespace ui
} //namespace mrrocpp
