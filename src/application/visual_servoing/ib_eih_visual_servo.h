/*
 * ib_eih_visual_servo.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef IB_EIH_VISUAL_SERVO_H_
#define IB_EIH_VISUAL_SERVO_H_

#include "visual_servo.h"
#include "IBReading.h"

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

	virtual Types::Mrrocpp_Proxy::IBReading* get_reading();

	virtual void reset();
protected:
	virtual lib::Homog_matrix compute_position_change(const lib::Homog_matrix& current_position, double dt);
	virtual bool is_object_visible_in_latest_reading();
	virtual void retrieve_reading();
	virtual void predict_reading();

	lib::Homog_matrix e_T_c_position;
private:
	Eigen::Matrix <double, 4, 1> desired_position;
	Types::Mrrocpp_Proxy::IBReading reading;
};

/** @} */

}//namespace generator

}

}

#endif /* IB_EIH_VISUAL_SERVO_H_ */
