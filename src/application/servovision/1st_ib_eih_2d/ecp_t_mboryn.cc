/*
 * $Id$
 * task/ecp_t_mboryn.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include "ecp_t_mboryn.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

const double
		ecp_t_mboryn::initialPositionJoints[MAX_SERVOS_NR] = { 0, -0.010, -1.693, -0.075, 0.011, 4.680, -1.577, 0.090 };

/*
 * IRP6 On Track:
 * XYZ AA
 * X: [0.590; 1.000], Y: [-0.20; 0.20], Z: 0.250
 *
 */

ecp_t_mboryn::ecp_t_mboryn(mrrocpp::lib::configurator& _configurator)
//: mrrocpp::ecp::common::task::task(_configurator)
:
	task(_configurator)
{
	ecp_m_robot = new ecp::irp6ot_m::robot(*this);
	ecp_g_mboryn_ = new generator::ecp_g_mboryn(*this);
	smooth_gen = new mrrocpp::ecp::common::generator::smooth(*this, true);

	//sr_ecp_msg->message("ecp_t_mboryn::ecp_t_mboryn() fradia setup...");
	vsp_fradia
			= new ecp_mp::sensor::fradia_sensor(lib::SENSOR_CVFRADIA, "[vsp_fradia_sensor_servovision]", *this, sizeof(lib::sensor_image_t::sensor_union_t::object_tracker_t));
	vsp_fradia->configure_sensor();

	sensor_m[lib::SENSOR_CVFRADIA] = vsp_fradia;
	ecp_g_mboryn_->sensor_m = sensor_m;
	//sr_ecp_msg->message("ecp_t_mboryn::ecp_t_mboryn() finished.");
}

ecp_t_mboryn::~ecp_t_mboryn()
{
	delete smooth_gen;
	delete ecp_g_mboryn_;
	delete ecp_m_robot;
}

void ecp_t_mboryn::main_task_algorithm(void)
{
	printf("ecp_t_mboryn::main_task_algorithm() begin\n");
	fflush(stdout);
	vsp_fradia->get_reading();
	while (vsp_fradia->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED) {
		vsp_fradia->get_reading();
	}

	moveToInitialPosition();

	printf("ecp_t_mboryn::main_task_algorithm() 1\n");
	fflush(stdout);
	ecp_g_mboryn_->Move();
	printf("ecp_t_mboryn::main_task_algorithm() 2\n");
	fflush(stdout);

	ecp_termination_notice();
}

void ecp_t_mboryn::moveToInitialPosition()
{
	smooth_gen->reset();
	smooth_gen->set_absolute();
	smooth_gen->load_coordinates(lib::ECP_JOINT, (double *) initialPositionJoints, true);
	smooth_gen->Move();
}

} // namespace task

} // namespace irp6ot

namespace common {

namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot::task::ecp_t_mboryn(_config);
}

} // namespace task

} // namespace common

} // namespace ecp

} // namespace mrrocpp
