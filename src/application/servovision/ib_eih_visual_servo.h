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

typedef ecp_mp::sensor::fradia_sensor <image_based_configuration, image_based_reading> ib_fradia_sensor;

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
			ib_eih_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, const std::string & section_name, mrrocpp::lib::configurator& configurator);
	virtual ~ib_eih_visual_servo();

	virtual boost::shared_ptr <ecp_mp::sensor::sensor_interface> get_vsp_fradia();
protected:
	virtual lib::Homog_matrix compute_position_change(const lib::Homog_matrix& current_position, double dt);
	virtual lib::sensor::VSP_REPORT_t get_sensor_report();
	virtual bool is_object_visible_in_latest_reading();
	boost::shared_ptr <ib_fradia_sensor> vsp_fradia;

	lib::Homog_matrix e_T_c_position;
private:
};

/** @} */

}//namespace generator

}

}

#endif /* IB_EIH_VISUAL_SERVO_H_ */
