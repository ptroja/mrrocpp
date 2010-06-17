/*
 * CaptureTask.cc
 *
 *  Created on: Apr 15, 2010
 *      Author: mboryn
 */

#include "capture_task.h"

#include "lib/logger.h"
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"

using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

CaptureTask::CaptureTask(mrrocpp::lib::configurator& configurator) :
	task(configurator) {
	log("CaptureTask::CaptureTask()\n");
	log_enabled = log_dbg_enabled = true;
	ecp_m_robot = new ecp::irp6ot_m::robot(*this);
	smoothGen = new mrrocpp::ecp::common::generator::smooth(*this, true);
	fradiaSensor = new capture_image_sensor(configurator, "[capture_task_fradia_config]");
	fradiaSensor->configure_sensor();
	et.captureNow = false;

	configurator.value<double> ("x_points", "[capture_image]");

	xPoints = configurator.value<int> ("x_points", "[capture_image]");
	yPoints = configurator.value<int> ("y_points", "[capture_image]");
	zPoints = configurator.value<int> ("z_points", "[capture_image]");

	deltaX = configurator.value<double> ("delta_x", "[capture_image]");
	deltaY = configurator.value<double> ("delta_y", "[capture_image]");
	deltaZ = configurator.value<double> ("delta_z", "[capture_image]");

	log("\n\nLiczba punktow zatrzymania x: %d, y: %d, z: %d\n", xPoints,
			yPoints, zPoints);
	log("Rozmiar kroku x: %g, y: %g, z: %g\n", deltaX, deltaY, deltaZ);
}

CaptureTask::~CaptureTask()
{
	delete fradiaSensor;
}

void CaptureTask::main_task_algorithm(void) {
	/*double v[MAX_SERVOS_NR] = { 0.20, 0.20, 0.01, 0.20, 0.20, 0.20, 0.20, 0.01 };
	 double a[MAX_SERVOS_NR] = { 0.15, 0.15, 0.5, 0.15, 0.15, 0.15, 0.15, 0.001 };

	 double initialPositionJoints[MAX_SERVOS_NR] = { 0, -0.010, -1.391, 0.222, 0.0, 4.719, 0.0, 0.075 };

	 smoothGen->reset();
	 smoothGen->set_absolute();
	 smoothGen->load_coordinates(lib::ECP_JOINT, v, a, initialPositionJoints, true);
	 smoothGen->Move();*/

	//	double v[MAX_SERVOS_NR] = { 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02 };
	//	double a[MAX_SERVOS_NR] = { 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02 };
	//
	//	smoothGen->reset();
	//	smoothGen->set_absolute();
	//	smoothGen->load_xyz_angle_axis(v, a, 0.925, 0, 0.05, -2.218, -2.218, 0, 0.075, 0, true);
	//	smoothGen->Move();

	log("\n");
	fflush(stdout);
	for (int i = 5; i; --i) {
		log("Ustaw obiekt (%d)...\r", i);
		sleep(1);
	}
	log("\n");

	smoothGen->reset();
	smoothGen->set_relative();

	et.captureNow = true;
	et.x = et.y = et.z = 0;

	//nextPosition(-deltaX * (xPoints - 1) / 2.0, -deltaY * (yPoints - 1) / 2.0, -deltaZ * (zPoints - 1) / 2.0);
	nextPosition(-deltaX * (xPoints - 1) / 2.0, -deltaY * (yPoints - 1) / 2.0,
			0);

	for (int k = 0; k < zPoints; ++k) {
		if (k > 0) {
			nextPosition(0, 0, -deltaZ);
		}
		for (int j = 0; j < yPoints; ++j) {
			if (j > 0) {
				nextPosition(0, deltaY, 0);
			}
			for (int i = 0; i < xPoints; ++i) {
				if (i > 0) {
					nextPosition(deltaX, 0, 0);
				}
				captureImage();
			}
			deltaX = -deltaX;
		}
		deltaY = -deltaY;
	}

	log("KONIEC\n");

	ecp_termination_notice();
}

void CaptureTask::nextPosition(double deltaX, double deltaY, double deltaZ) {
	double
			v[MAX_SERVOS_NR] = { 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02 };
	double
			a[MAX_SERVOS_NR] = { 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02 };

	et.x += deltaX;
	et.y += deltaY;
	et.z += deltaZ;

	smoothGen->load_xyz_angle_axis(v, a, deltaX, deltaY, deltaZ, 0, 0, 0, 0, 0,
			true);
	smoothGen->Move();
}

void CaptureTask::captureImage() {
	sleep(2);
	fradiaSensor->set_initiate_message(et);
	sleep(2);
}

}// namespace task

}// namespace irp6ot


namespace common {

namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6ot_m::task::CaptureTask(_config);
}

} // namespace task

} // namespace common

}//namespace ecp

}// namespace mrrocpp
