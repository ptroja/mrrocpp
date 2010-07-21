/*
 * pb_sac_visual_servo.h
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#ifndef PB_SAC_VISUAL_SERVO_H_
#define PB_SAC_VISUAL_SERVO_H_

#include "pb_visual_servo.h"

using visual_servo_types::position_based_configuration;
using visual_servo_types::position_based_reading;

namespace mrrocpp {

namespace ecp {

namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class pb_sac_visual_servo : public pb_visual_servo
{
public:
	pb_sac_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, const std::string& section_name, mrrocpp::lib::configurator& configurator);
	virtual ~pb_sac_visual_servo();

protected:
	virtual lib::Homog_matrix compute_position_change(const lib::Homog_matrix& current_position, double dt);

	lib::Homog_matrix O_T_C;
};

/** @} */

} //namespace

}

}

#endif /* PB_SAC_VISUAL_SERVO_H_ */
