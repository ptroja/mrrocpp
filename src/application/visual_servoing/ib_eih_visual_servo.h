/*
 * ib_eih_visual_servo.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef IB_EIH_VISUAL_SERVO_H_
#define IB_EIH_VISUAL_SERVO_H_

#include "visual_servo.h"
#include "visual_servo_types.h"

using visual_servo_types::image_based_reading;
using visual_servo_types::image_based_configuration;

namespace mrrocpp {

namespace ecp {

namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class ib_eih_visual_servo : public visual_servo
{
public:
	ib_eih_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor, const std::string & section_name, mrrocpp::lib::configurator& configurator);
	virtual ~ib_eih_visual_servo();

protected:
	virtual lib::Homog_matrix compute_position_change(const lib::Homog_matrix& current_position, double dt);
	virtual bool is_object_visible_in_latest_reading();

	lib::Homog_matrix e_T_c_position;
private:
};

/** @} */

}//namespace generator

}

}

#endif /* IB_EIH_VISUAL_SERVO_H_ */
