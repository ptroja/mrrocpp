/*
 * ecp_t_pb_sac_calibration.cc
 *
 *  Created on: 14-12-2010
 *      Author: mboryn
 */


#include <algorithm>

#include "ecp_t_pb_sac_calibration.h"
#include "base/lib/logger.h"
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

using boost::shared_ptr;

ecp_t_pb_sac_calibration::ecp_t_pb_sac_calibration(mrrocpp::lib::configurator& config) :
	mrrocpp::ecp::common::task::task(config)
{
	log_enabled = true;
	log_dbg_enabled = true;
	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);

	string config_section_name = "[sac_calibration]";

	ds = boost::shared_ptr <discode_sensor>(new discode_sensor(config, config_section_name));

	vs = boost::shared_ptr <pb_sac_calibration>(new pb_sac_calibration(ds, config_section_name, config));

	term_cond = boost::shared_ptr <termination_condition> (new timeout_termination_condition(10));

	sm
			= boost::shared_ptr <single_visual_servo_manager>(new single_visual_servo_manager(*this, config_section_name.c_str(), vs));
	sm->add_termination_condition(term_cond);
	sm->configure();

	log_dbg("\necp_t_pb_sac_calibration::ecp_t_pb_sac_calibration() end\n");
}

ecp_t_pb_sac_calibration::~ecp_t_pb_sac_calibration()
{
}

void ecp_t_pb_sac_calibration::main_task_algorithm()
{
	log_dbg("\necp_t_pb_sac_calibration::main_task_algorithm() begin\n");

	sm->Move();

	log("=======================================================\n");
	log("=======================================================\n");
	vector<lib::Homog_matrix> all_O_T_C = vs->get_all_O_T_C();
	log("all_O_T_C.size() = %d\n", (int)all_O_T_C.size());
	int n = min((int)all_O_T_C.size(), 10);
	for(int i=0; i<n; ++i){
		log("%d: -------------------------------------------------------\n", i);
		log(all_O_T_C[i]);
		log("-------------------------------------------------------\n");
	}
	log("=======================================================\n");
	log("=======================================================\n");


	log_dbg("\necp_t_pb_sac_calibration::main_task_algorithm() end\n");

	ecp_termination_notice();
}

} // namespace task
} // namespace irp6p_m

namespace common {
namespace task {
task_base* return_created_ecp_task(lib::configurator &config)
{
	return new irp6p_m::task::ecp_t_pb_sac_calibration(config);
}
} // namespace task
} // namespace common

} // namespace ecp
} // namespace mrrocpp
