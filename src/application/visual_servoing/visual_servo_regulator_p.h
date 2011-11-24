/*
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_REGULATOR_P_H_
#define VISUAL_SERVO_REGULATOR_P_H_

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
class regulator_p : public visual_servo_regulator
{
public:
	regulator_p(const lib::configurator & config, const std::string& config_section_name);

	virtual ~regulator_p();

	virtual const Eigen::Matrix <double, 6, 1> & compute_control(const Eigen::Matrix <double, 6, 1> & error, double dt);
	virtual void reset();

	Eigen::Matrix <double, 6, 6> Kp;
protected:

}; // class regulator_p

/** @} */

} // namespace servovision
}
}

#endif /* VISUAL_SERVO_REGULATOR_P_H_ */
