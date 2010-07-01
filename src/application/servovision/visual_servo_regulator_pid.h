/*
 * visual_servo_regulator_pid.h
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_REGULATOR_PID_H_
#define VISUAL_SERVO_REGULATOR_PID_H_

#include "visual_servo_regulator.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class regulator_pid : public visual_servo_regulator
{
public:
	regulator_pid(const lib::configurator & config, const std::string& config_section_name);
	virtual ~regulator_pid();

	virtual const Eigen::Matrix <double, 6, 1>
			& compute_control(const Eigen::Matrix <double, 6, 1> & error, double dt);
protected:
	Eigen::Matrix <double, 6, 6> Kp, Ki, Kd;
	Eigen::Matrix <double, 6, 1> error_t_1;
	Eigen::Matrix <double, 6, 1> error_t_2;
};

/** @} */

} // namespace
}
}

#endif /* VISUAL_SERVO_REGULATOR_PID_H_ */
