/*!
 * @file
 * @brief File contains wgt_sbench_voltage_command class definition for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "../sbench/ui_ecp_r_sbench.h"
#include "../sbench/ui_r_sbench.h"

#include "wgt_sbench_voltage_command.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

wgt_sbench_voltage_command::wgt_sbench_voltage_command(const QString & _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		wgt_sbench_command(_widget_label, _interface, _robot, parent)
{
}

void wgt_sbench_voltage_command::read_and_set()
{
	init();

	wgt_sbench_command::set(robot->ui_ecp_robot->the_robot->data_request_port.data.voltage_buf);

}

void wgt_sbench_voltage_command::execute()
{
	wgt_sbench_command::get(robot->ui_ecp_robot->the_robot->power_supply_data_port.data);

	robot->ui_ecp_robot->the_robot->power_supply_data_port.set();
	robot->ui_ecp_robot->the_robot->data_request_port.set_request();
	robot->ui_ecp_robot->execute_motion();
	robot->ui_ecp_robot->the_robot->data_request_port.get();

	reshresh_widgets();
}

void wgt_sbench_voltage_command::reshresh_widgets()
{
	refresh_dock_widgets(robot->ui_ecp_robot->the_robot->data_request_port.data.voltage_buf);
}

