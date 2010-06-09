/*
 * CaptureTask.h
 *
 *  Created on: Apr 15, 2010
 *      Author: mboryn
 */

#ifndef CAPTURETASK_H_
#define CAPTURETASK_H_

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {


struct effectorTranslation {
	bool captureNow;
	double x;
	double y;
	double z;
};


typedef mrrocpp::ecp_mp::sensor::fradia_sensor <lib::empty_t, lib::empty_t, effectorTranslation> capture_image_sensor;


class CaptureTask: public mrrocpp::ecp::common::task::task
{
public:
	CaptureTask(mrrocpp::lib::configurator& configurator);
	virtual ~CaptureTask();
	void main_task_algorithm(void);
protected:
	mrrocpp::ecp::common::generator::smooth* smoothGen;
	capture_image_sensor* fradiaSensor;

	effectorTranslation et;

	void nextPosition(double deltaX, double deltaY, double deltaZ);
	void captureImage();

	double v[MAX_SERVOS_NR];
	double a[MAX_SERVOS_NR];

	int xPoints, yPoints, zPoints;

	double deltaX;
	double deltaZ;
	double deltaY;
};

}// namespace task

}

}

} // namespace mrrocpp

#endif /* CAPTURETASK_H_ */
