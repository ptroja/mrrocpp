/*
 * bclikeregions.cc
 *
 *  Created on: May 18, 2010
 *      Author: kszkudla
 */

#include "bclikeregions_task.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

bclikeregions_task::bclikeregions_task(mrrocpp::lib::configurator& configurator):task(configurator) {

	// TODO Auto-generated constructor stub

	ecp_m_robot = new ecp::irp6ot_m::robot(*this);
	//gen = shared_ptr<generator::bclikeregions_gen> (new generator::bclikeregions_gen(*this));
	bc_smooth = shared_ptr<generator::bclike_gen> (new generator::bclike_gen(*this));

//	sensor_m[ecp_mp::sensor::SENSOR_CVFRADIA] = new bcl_fradia_sensor(this->config, "[vsp_fradia_sensor]");
//	sensor_m[ecp_mp::sensor::SENSOR_CVFRADIA]->configure_sensor();

}

bclikeregions_task::~bclikeregions_task() {

	// TODO Auto-generated destructor stub

}

void bclikeregions_task::main_task_algorithm(void){

	//gen->Move();
	bc_smooth->set_absolute();

//	bc_smooth->load_coordinates(lib::ECP_JOINT, 0.0, 0.5, -1.87, 0.100, -0.040, 4.627, -1.57, 0.0, true);
	double tmp[] = { 0.0, 0.5, -1.87, 0.100, -0.040, 4.627, -1.57, 0.0};
	std::vector<double> vec(tmp, tmp + 8);
	bc_smooth->load_absolute_joint_trajectory_pose(vec);
	bc_smooth->calculate_interpolate();
//	bc_smooth->load_file_with_path(
//			"/home/kszkudla/workspace/mrrocpp/src/application/bclikeregions/trj/trj_left.trj");
	bc_smooth->Move();
	bc_smooth->reset();
	termination_notice();

}

boost::shared_ptr <bcl_fradia_sensor> bclikeregions_task::get_vsp_fradia(){
	return vsp_fradia;
}

task* return_created_ecp_task(lib::configurator &config){

	return new bclikeregions_task(config);

}

}

}

}

}


///*
// * bclikeregions.cc
// *
// *  Created on: May 18, 2010
// *      Author: kszkudla
// */
//
//#include "application/bclikeregions/bclikeregions_task.h"
//#include "application/bclikeregions/bclikeregions_gen.h"
//
//#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
//
//#include <boost/shared_ptr.hpp>
//#include "base/ecp/ecp_task.h"
//#include "bclikeregions_gen.h"
//#include "base/ecp/ecp_robot.h"
//
//#include "lib/typedefs.h"
//#include "lib/impconst.h"
//#include "lib/com_buf.h"
//#include "lib/srlib.h"
//
//#include "robot/irp6ot_m/irp6ot_m_const.h"
//#include "robot/irp6p_m/irp6p_m_const.h"
//
//
//namespace mrrocpp {
//
//namespace mp{
//
//namespace task {
//
//
//
//bclikeregions_task::bclikeregions_task(mrrocpp::lib::configurator& configurator):task(configurator) {
//
//	// TODO Auto-generated constructor stub
//
//	//ecp_m_robot = new ecp::irp6ot::robot(*this);
//	//gen1 = shared_ptr<generator::bclikeregions_gen> (new generator::bclikeregions_gen(*this));
//
//
//
//}
//
//
//bclikeregions_task::~bclikeregions_task() {
//
//	// TODO Auto-generated destructor stub
//
//}
//
//void bclikeregions_task::main_task_algorithm(void){
//
////	gen1->Move();
////	termination_notice();
//
//}
//
//void bclikeregions_task::configure_edp_force_sensor(bool configure_track, bool configure_postument)
//{
//	if (configure_track) {
//		set_next_ecp_state(ecp_mp::task::ECP_ST_BIAS_EDP_FORCE, 0, "", 0, lib::ROBOT_IRP6OT_M);
//	}
//
//	if (configure_postument) {
//		set_next_ecp_state(ecp_mp::task::ECP_ST_BIAS_EDP_FORCE, 0, "", 0, lib::ROBOT_IRP6P_M);
//	}
//
//}
//
//task* return_created_ecp_task(lib::configurator &config){
//
//	return new bclikeregions_task(config);
//
//}
//
//
//}
//}
//}

