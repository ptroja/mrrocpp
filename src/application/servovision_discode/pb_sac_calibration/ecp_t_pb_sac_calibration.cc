/*
 * ecp_t_pb_sac_calibration.cc
 *
 *  Created on: 14-12-2010
 *      Author: mboryn
 */

#include "ecp_t_pb_sac_calibration.h"

#include "base/lib/logger.h"

#include "pb_sac_calibration.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {
namespace task {

using namespace logger;
using namespace std;
using namespace mrrocpp::ecp::servovision;
using mrrocpp::ecp_mp::sensor::discode::discode_sensor;
using mrrocpp::ecp::common::generator::single_visual_servo_manager;

ecp_t_pb_sac_calibration::ecp_t_pb_sac_calibration(mrrocpp::lib::configurator& config) :
	task(config)
{
	log_dbg_enabled = true;
	log_dbg("\necp_t_pb_sac_calibration::ecp_t_pb_sac_calibration() begin\n");
	ecp_m_robot = new ecp::irp6p_m::robot(*this);

	log_dbg("\necp_t_pb_sac_calibration::ecp_t_pb_sac_calibration() 1\n");
	string config_section_name = "[sac_calibration]";

	ds = boost::shared_ptr <discode_sensor>(new discode_sensor(config, config_section_name));

	log_dbg("\necp_t_pb_sac_calibration::ecp_t_pb_sac_calibration() 2\n");

	vs = boost::shared_ptr <visual_servo>(new pb_sac_calibration(ds, config_section_name, config));

	log_dbg("\necp_t_pb_sac_calibration::ecp_t_pb_sac_calibration() 3\n");

	sm = boost::shared_ptr <single_visual_servo_manager>(new single_visual_servo_manager(*this, config_section_name.c_str(), vs));

	log_dbg("\necp_t_pb_sac_calibration::ecp_t_pb_sac_calibration() 4\n");
}

ecp_t_pb_sac_calibration::~ecp_t_pb_sac_calibration()
{
	// TODO Auto-generated destructor stub
}

void ecp_t_pb_sac_calibration::main_task_algorithm()
{
	log_dbg("\necp_t_pb_sac_calibration::main_task_algorithm() begin\n");
	sm->Move();
	log_dbg("\necp_t_pb_sac_calibration::main_task_algorithm() end\n");
	ecp_termination_notice();
}

} // namespace task
} // namespace irp6p_m

namespace common {
namespace task {
task* return_created_ecp_task(lib::configurator &config)
{
	return new irp6p_m::task::ecp_t_pb_sac_calibration(config);
}
} // namespace task
} // namespace common

} // namespace ecp
} // namespace mrrocpp
