#include "signal_dispatcher.h"
#include "interface.h"
#include "../base/ui_robot.h"
#include "../irp6ot_m/ui_r_irp6ot_m.h"
#include <boost/foreach.hpp>
#include "wgt_base.h"
#include "wgt_robot_process_control.h"

namespace Ui {

SignalDispatcher::SignalDispatcher(mrrocpp::ui::common::Interface& iface) :
		interface(iface)
{
}

SignalDispatcher::~SignalDispatcher()
{
}

void SignalDispatcher::on_EDP_Load_triggered(mrrocpp::ui::common::UiRobot *robot)
{
	interface.get_main_window()->ui_robot_action(robot, &mrrocpp::ui::common::UiRobot::edp_create);
}

void SignalDispatcher::on_EDP_Unload_triggered(mrrocpp::ui::common::UiRobot *robot)
{
	robot->delete_robot_process_control_window();
	interface.get_main_window()->ui_robot_action(robot, &mrrocpp::ui::common::UiRobot::EDP_slay_int);
}

void SignalDispatcher::on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot *robot)
{
	interface.get_main_window()->ui_robot_action(robot, &mrrocpp::ui::common::UiRobot::synchronise);
}

void SignalDispatcher::on_Clear_Fault_triggered(mrrocpp::ui::common::UiRobot *robot)
{
	interface.get_main_window()->ui_robot_action(robot, &mrrocpp::ui::common::UiRobot::execute_clear_fault);
}

void SignalDispatcher::on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot *robot)
{
	interface.get_main_window()->ui_robot_action(robot, &mrrocpp::ui::common::UiRobot::move_to_synchro_position);
}

void SignalDispatcher::on_Front_Position_triggered(mrrocpp::ui::common::UiRobot *robot)
{
	interface.get_main_window()->ui_robot_action(robot, &mrrocpp::ui::common::UiRobot::move_to_front_position);
}

void SignalDispatcher::on_Position_0_triggered(mrrocpp::ui::common::UiRobot *robot)
{

	interface.get_main_window()->ui_robot_action(robot, &mrrocpp::ui::common::UiRobot::move_to_preset_position, 0);
}

void SignalDispatcher::on_Position_1_triggered(mrrocpp::ui::common::UiRobot *robot)
{
	interface.get_main_window()->ui_robot_action(robot, &mrrocpp::ui::common::UiRobot::move_to_preset_position, 1);
}

void SignalDispatcher::on_Position_2_triggered(mrrocpp::ui::common::UiRobot *robot)
{
	interface.get_main_window()->ui_robot_action(robot, &mrrocpp::ui::common::UiRobot::move_to_preset_position, 2);
}

void SignalDispatcher::open_new_window(wgt_base *window, bool set_on_top)
{
	interface.get_main_window()->open_new_window(window, &wgt_base::my_open, set_on_top);
}

}
