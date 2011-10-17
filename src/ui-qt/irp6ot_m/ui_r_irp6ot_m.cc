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

void UiRobot::setup_menubar()
{
	irp6_m::UiRobot::setup_menubar();

	robot_menu->setTitle(QApplication::translate("MainWindow", "Irp6&ot_m", 0, QApplication::UnicodeUTF8));

}

int UiRobot::synchronise()
{
	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::synchronise_int, &(*this)));
	return 1;
}

}
} //namespace ui
} //namespace mrrocpp
