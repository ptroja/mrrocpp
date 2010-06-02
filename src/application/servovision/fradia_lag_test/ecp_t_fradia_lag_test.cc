/*
 * $Id: ecp_t_mboryn.cc 3480 2010-01-08 18:48:29Z mboryn $
 * task/ecp_t_mboryn.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include "ecp_t_fradia_lag_test.h"

#include <signal.h>
#include <time.h>
#include <err.h>

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

const double
		ecp_t_fradia_lag_test::initialPositionJoints[MAX_SERVOS_NR] = { 0, -0.010, -1.693, -0.075, 0.011, 4.680, -1.577, 0.090 };

/*
 * IRP6 On Track:
 * XYZ AA
 * X: [0.590; 1.000], Y: [-0.20; 0.20], Z: 0.250
 *
 */

ecp_t_fradia_lag_test::ecp_t_fradia_lag_test(mrrocpp::lib::configurator& _configurator)
//: mrrocpp::ecp::common::task::task(_configurator)
:
	task(_configurator)
{
	ecp_m_robot = new ecp::irp6ot_m::robot(*this);
	smooth_gen = new mrrocpp::ecp::common::generator::smooth(*this, true);

	//sr_ecp_msg->message("ecp_t_fradia_lag_test::ecp_t_fradia_lag_test() fradia setup...");
	vsp_fradia
			= new ecp_mp::sensor::fradia_sensor(lib::SENSOR_CVFRADIA, "[vsp_fradia_sensor_servovision]", *this, sizeof(lib::sensor_image_t::sensor_union_t::object_tracker_t));
	vsp_fradia->configure_sensor();

	sensor_m[lib::SENSOR_CVFRADIA] = vsp_fradia;
	//sr_ecp_msg->message("ecp_t_fradia_lag_test::ecp_t_fradia_lag_test() finished.");
}

ecp_t_fradia_lag_test::~ecp_t_fradia_lag_test()
{
	delete smooth_gen;
	delete ecp_m_robot;
}

void ecp_t_fradia_lag_test::main_task_algorithm(void)
{
	printf("ecp_t_fradia_lag_test::main_task_algorithm() begin\n");
	fflush(stdout);

	vsp_fradia->get_reading();
	while (vsp_fradia->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED) {
		vsp_fradia->get_reading();
	}

	moveToInitialPosition();

	printf("ecp_t_fradia_lag_test::main_task_algorithm() 1\n");
	fflush(stdout);

	// start timer
	struct timespec startTime, stopTime;

	if (clock_gettime(CLOCK_REALTIME, &startTime) == -1) {
		perror("clock gettime");
	}

	// do the communication
	int maxMessages = 5000;
	const int counters_sz = 8;
	int counters[counters_sz];
	for (int i = 0; i < counters_sz; ++i) {
		counters[i] = 0;
	}

	vsp_fradia->get_reading();

	int prev_msg_number = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.z;
	int first_msg_number = prev_msg_number, last_msg_number;
	int curr_msg_number = 0;
	for (int i = 0; i < maxMessages; ++i) {
		vsp_fradia->get_reading();
		switch (vsp_fradia->from_vsp.vsp_report)
		{
			case lib::VSP_REPLY_OK:
				counters[0]++;
				break;
			case lib::VSP_SENSOR_NOT_CONFIGURED:
				counters[1]++;
				break;
			case lib::VSP_READING_NOT_READY:
				counters[2]++;
				break;
			case lib::INVALID_VSP_COMMAND:
				counters[3]++;
				break;
			default:
				counters[4]++;
		}

		curr_msg_number = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.z;
		if (curr_msg_number == prev_msg_number + 1) { // number of consecutive readings
			counters[5]++;
		}
		if (curr_msg_number != prev_msg_number) { // total number of unique readings
			counters[6]++;
		}
		if (curr_msg_number > prev_msg_number) { // total number of unique readings
			counters[7]++;
		}
		prev_msg_number = curr_msg_number;
	}
	last_msg_number = curr_msg_number;
	//usleep(2000000);

	if (clock_gettime(CLOCK_REALTIME, &stopTime) == -1) {
		perror("clock gettime");
	}

	double deltaT = (stopTime.tv_sec - startTime.tv_sec) + (double) (stopTime.tv_nsec - startTime.tv_nsec)
			/ (double) 1e9L;

	printf("total time: %g s; average time per reading: %g s\n", deltaT, deltaT / maxMessages);
	for (int i = 0; i < counters_sz; ++i) {
		printf("counters[%d] = %d\n", i, counters[i]);
	}

	printf("first_msg_number = %d; last_msg_number = %d\n", first_msg_number, last_msg_number);

	printf("ecp_t_fradia_lag_test::main_task_algorithm() 2\n");
	fflush(stdout);

	ecp_termination_notice();
}

void ecp_t_fradia_lag_test::moveToInitialPosition()
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
	return new irp6ot::task::ecp_t_fradia_lag_test(_config);
}

} // namespace task

} // namespace common

} // namespace ecp

} // namespace mrrocpp
