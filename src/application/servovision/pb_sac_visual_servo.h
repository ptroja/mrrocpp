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

namespace common {

namespace generator {

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

	virtual lib::Homog_matrix get_position_change(const lib::Homog_matrix& current_position, double dt);
	virtual boost::shared_ptr <ecp_mp::sensor::sensor_interface> get_vsp_fradia();
private:
	boost::shared_ptr <pb_fradia_sensor> vsp_fradia;

	lib::Homog_matrix G_T_E_desired;
	lib::Homog_matrix O_T_C;
};

/** @} */

} //namespace

}

}

}

#endif /* PB_SAC_VISUAL_SERVO_H_ */
