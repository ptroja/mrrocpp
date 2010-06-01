/*
 * $Id: ecp_t_mboryn.cc 3480 2010-01-08 18:48:29Z mboryn $
 * task/ecp_t_mboryn.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include <stdexcept>

#include "ecp_t_ib_eih_tester.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

const double
		ecp_t_ib_eih_tester::initialPositionJoints[MAX_SERVOS_NR] = { 0, -0.010, -1.391, 0.222, 0.0, 4.719, 0.0, 0.075};

//{ 0, -0.010, -1.510, -0.113, 0.0, 4.716, 0.0, 0.075};

/*
 * IRP6 On Track:
 * XYZ AA
 * X: [0.590; 1.000], Y: [-0.20; 0.20], Z: 0.250
 *
 */

ecp_t_ib_eih_tester::ecp_t_ib_eih_tester(mrrocpp::lib::configurator& _configurator)
//: mrrocpp::ecp::common::task::task(_configurator)
:
	task(_configurator)
{
	try {
		ecp_m_robot = new ecp::irp6ot_m::robot(*this);

		smooth_gen = new mrrocpp::ecp::common::generator::smooth(*this, true);

		sr_ecp_msg->message("ecp_t_ib_eih_tester::ecp_t_ib_eih_tester() fradia setup...");

		vsp_fradia
				= new ecp_mp::sensor::fradia_sensor <ecp::common::generator::visual_object_tracker, char>("[vsp_cvfradia_servovision]", *this);

		vsp_fradia->configure_sensor();

		regulator = new ecp::common::generator::regulator_p <4, 4>(_configurator, "[regulator_p]");

		g_ib_eih = new ecp::common::generator::ecp_g_ib_eih(*this, vsp_fradia, regulator);

		befgen = new common::generator::bias_edp_force(*this);
		gagen = new common::generator::tff_gripper_approach(*this, 8); //gripper approach constructor (task&, no_of_steps)

		sensor_m[lib::SENSOR_CVFRADIA] = vsp_fradia;
		//g_ib_eih->sensor_m = sensor_m;
		sr_ecp_msg->message("ecp_t_ib_eih_tester::ecp_t_ib_eih_tester() finished.");
	} catch (const std::exception & e) {
		sr_ecp_msg->message(e.what());
		throw e;
	}
}

ecp_t_ib_eih_tester::~ecp_t_ib_eih_tester()
{
	delete smooth_gen;
	delete g_ib_eih;
	delete ecp_m_robot;
	delete befgen;
	delete gagen;
}

void ecp_t_ib_eih_tester::main_task_algorithm(void)
{
	printf("ecp_t_ib_eih_tester::main_task_algorithm() begin\n");
	fflush(stdout);

	moveToInitialPosition();

	printf("ecp_t_ib_eih_tester::main_task_algorithm() 1\n");
	fflush(stdout);

//	while(1){
		g_ib_eih->Move();
//	}

	printf("ecp_t_ib_eih_tester::main_task_algorithm() 2\n");
	fflush(stdout);

	befgen->Move();
	gagen->configure(0.01, 1000);
	gagen->Move();

	printf("ecp_t_ib_eih_tester::main_task_algorithm() 3\n");
	fflush(stdout);

	double v[MAX_SERVOS_NR] = { 0.20, 0.20, 0.01, 0.20, 0.20, 0.20, 0.001, 0.003 };
	double a[MAX_SERVOS_NR] = { 0.15, 0.15, 0.5, 0.15, 0.15, 0.15, 0.001, 0.001 };

	smooth_gen->set_relative();
	smooth_gen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, v, a, 0, 0, -0.005, 0, 0, 0, 0, 0, true);
	smooth_gen->Move();
	smooth_gen->reset();

	smooth_gen->load_coordinates(lib::ECP_JOINT, v, a, 0, 0, 0, 0, 0, 0, 0, -0.017, true); //close gripper
	smooth_gen->Move();
	smooth_gen->reset();

	smooth_gen->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, v, a, 0, 0, -0.1, 0, 0, 0, 0, 0, true);
	smooth_gen->Move();
	smooth_gen->reset();

	ecp_termination_notice();
}

void ecp_t_ib_eih_tester::moveToInitialPosition()
{
	double v[MAX_SERVOS_NR] = { 0.20, 0.20, 0.01, 0.20, 0.20, 0.20, 0.20, 0.01 };
	double a[MAX_SERVOS_NR] = { 0.15, 0.15, 0.5, 0.15, 0.15, 0.15, 0.15, 0.001 };

	smooth_gen->reset();
	smooth_gen->set_absolute();
	smooth_gen->load_coordinates(lib::ECP_JOINT, v, a, (double *) initialPositionJoints, true);
	smooth_gen->Move();
}

} // namespace task

} // namespace irp6ot

namespace common {

namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot::task::ecp_t_ib_eih_tester(_config);
}

} // namespace task

} // namespace common

} // namespace ecp

} // namespace mrrocpp
