/*
 * single_visual_servo_manager.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef SINGLE_VISUAL_SERVO_MANAGER_H_
#define SINGLE_VISUAL_SERVO_MANAGER_H_

#include "visual_servo_manager.h"

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
class single_visual_servo_manager : public visual_servo_manager
{
public:
			single_visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
					mrrocpp::ecp::servovision::visual_servo> vs);
	virtual ~single_visual_servo_manager();
protected:
	virtual lib::Homog_matrix get_aggregated_position_change();
	virtual void configure_all_servos();
};

/** @} */

}//namespace generator

}

}

}

#endif /* SINGLE_VISUAL_SERVO_MANAGER_H_ */
