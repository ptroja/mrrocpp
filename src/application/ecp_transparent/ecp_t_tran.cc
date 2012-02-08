/*!
 * @file
 * @brief File contains ecp transparent task definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup
 */

#include <cstdio>
#include <cstring>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "robot/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_tfg/ecp_r_irp6p_tfg.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "robot/conveyor/ecp_r_conv.h"

#include "robot/irp6p_m/const_irp6p_m.h"

#include "ecp_t_tran.h"
#include "generator/ecp/transparent/ecp_g_transparent.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
tran::tran(lib::configurator &_config) :
		common::task::task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_tfg::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_tfg::robot(*this);
	} else if (config.robot_name == lib::irp6p_tfg::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_tfg::robot(*this);
	} else if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else if (config.robot_name == lib::conveyor::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new conveyor::robot(*this);
	}

	sr_ecp_msg->message("ecp loaded");
}

void tran::main_task_algorithm(void)
{
	generator::transparent gt(*this);
	sr_ecp_msg->message("Ruch");

	gt.Move();
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new tran(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
