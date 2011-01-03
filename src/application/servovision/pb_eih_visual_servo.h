/*
 * pb_eih_visual_servo.h
 *
 *  Created on: May 11, 2010
 *      Author: mboryn
 */

#ifndef PB_EIH_VISUAL_SERVO_H_
#define PB_EIH_VISUAL_SERVO_H_

#include "pb_visual_servo.h"

#include "vs_logger.h"

namespace mrrocpp {

namespace ecp {

namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class pb_eih_visual_servo : public pb_visual_servo
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	pb_eih_visual_servo( boost::shared_ptr <visual_servo_regulator> regulator, const std::string& section_name, mrrocpp::lib::configurator& configurator);
	virtual ~pb_eih_visual_servo();
protected:
	virtual lib::Homog_matrix compute_position_change(const lib::Homog_matrix& current_position, double dt);

	lib::Homog_matrix E_T_C;
};

/** @} */

} // namespace generator

}

}


#endif /* PB_EIH_VISUAL_SERVO_H_ */
