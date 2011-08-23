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
#include "../base/mp.h"
#include "../base/ui_robot.h"
#include <boost/foreach.hpp>

namespace mrrocpp {
namespace ui {
namespace irp6ot_m {
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
	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_motor_motion, &(*this)));

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
	add_wgt <wgt_irp6_m_joints>(WGT_JOINTS, "Irp6ot_m joints");
	add_wgt <wgt_irp6_m_motors>(WGT_MOTORS, "Irp6ot_m motors");
	add_wgt <wgt_irp6_m_angle_axis>(WGT_ANGLE_AXIS, "Irp6ot_m angle axis");
	add_wgt <wgt_irp6_m_euler>(WGT_EULER, "Irp6ot_m euler");
	add_wgt <wgt_irp6_m_relative_angle_axis>(WGT_RELATIVE_ANGLE_AXIS, "Irp6ot_m relative angle axis");
	add_wgt <wgt_irp6_m_tool_angle_axis>(WGT_TOOL_ANGLE_AXIS, "Irp6ot_m tool angle axis");
	add_wgt <wgt_irp6_m_tool_euler>(WGT_TOOL_EULER, "Irp6ot_m tool euler");

}

void UiRobot::make_connections()
{
//	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();
//
//	connect(actionirp6ot_m_Synchronisation, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), 				Qt::AutoCompatConnection);
//	connect(actionirp6ot_m_Synchro_Position,SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)),				Qt::AutoCompatConnection);
//	connect(actionirp6ot_m_Front_Position, 	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Front_Position_triggered(mrrocpp::ui::common::UiRobot*)), 				Qt::AutoCompatConnection);
//	connect(actionirp6ot_m_Position_0, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
//	connect(actionirp6ot_m_Position_1, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
//	connect(actionirp6ot_m_Position_2, 		SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), 					Qt::AutoCompatConnection);
}

void UiRobot::setup_menubar()
{
	irp6_m::UiRobot::setup_menubar();
//	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
//	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	robot_menu->setTitle(QApplication::translate("MainWindow", "Irp6&ot_m", 0, QApplication::UnicodeUTF8));

	make_connections();
}

int UiRobot::synchronise()
{
	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::synchronise_int, &(*this)));
	return 1;
}

int UiRobot::manage_interface()
{
	//MainWindow *mw = interface.get_main_window();
	irp6_m::UiRobot::manage_interface();

	return 1;
}

// aktualizacja ustawien przyciskow
int UiRobot::process_control_window_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{
	//wgt_process_control *ui = interface.get_process_control_window(); //TODO: zmienic sposob

	if (state.edp.state <= 0) { // edp wylaczone
//		ui->all_reader_start_pushButton->setDisabled(true);
//		ui->all_reader_stop_pushButton->setDisabled(true);
//		ui->all_reader_trigger_pushButton->setDisabled(true);

		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		 */
	} else if (state.edp.state == 1) { // edp wlaczone reader czeka na start
		wlacz_PtButton_wnd_processes_control_all_reader_start = true;
//		ui->all_reader_start_pushButton->setEnabled(true);
//		ui->all_reader_stop_pushButton->setDisabled(true);
//		ui->all_reader_trigger_pushButton->setDisabled(true);
		/* TR
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		 */
	} else if (state.edp.state == 2) { // edp wlaczone reader czeka na stop
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
