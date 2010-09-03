/*
 * object_reached_termination_condition.cc
 *
 *  Created on: Apr 29, 2010
 *      Author: mboryn
 */

#include "object_reached_termination_condition.h"
#include "base/lib/logger.h"

using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace servovision {

object_reached_termination_condition::object_reached_termination_condition(const lib::configurator& config, const std::string &section_name)
{
	max_linear_speed = config.value <double> ("max_linear_speed", section_name);
	max_angular_speed = config.value <double> ("max_angular_speed", section_name);
	max_linear_accel = config.value <double> ("max_linear_accel", section_name);
	max_angular_accel = config.value <double> ("max_angular_accel", section_name);
	max_linear_error = config.value <double> ("max_linear_error", section_name);
	max_angular_error = config.value <double> ("max_angular_error", section_name);
	min_steps = config.value <int> ("min_steps", section_name);
	steps_delay = 0;
}

object_reached_termination_condition::~object_reached_termination_condition()
{
}

void object_reached_termination_condition::reset()
{
	steps_delay = 0;
	condition_met = false;
}

void object_reached_termination_condition::update(const mrrocpp::ecp::common::generator::visual_servo_manager* vsm)
{
	bool object_visible_and_error_small = false;
	for (int i = 0; i < vsm->get_servos().size(); ++i) {
		Eigen::Matrix <double, 6, 1> e = vsm->get_servos()[i]->get_error();
		Eigen::Matrix <double, 3, 1> linear_error = e.block(0, 0, 3, 1);
		Eigen::Matrix <double, 3, 1> angular_error = e.block(3, 0, 3, 1);

//		log_dbg("object_reached_termination_condition: vsm->get_servos()[i]->is_object_visible() = %d\n", (int) vsm->get_servos()[i]->is_object_visible());
//		log_dbg("object_reached_termination_condition: linear_error.norm() = %g\n", linear_error.norm());
//		log_dbg("object_reached_termination_condition: angular_error.norm() = %g\n", angular_error.norm());

		if (vsm->get_servos()[i]->is_object_visible() && linear_error.norm() <= max_linear_error
				&& angular_error.norm() <= max_angular_error) {
			object_visible_and_error_small = true;
		}
	}

//	log_dbg("object_reached_termination_condition: object_visible_and_error_small = %d\n", (int) object_visible_and_error_small);
//	log_dbg("object_reached_termination_condition: vsm->get_linear_speed() = %g\n", vsm->get_linear_speed());
//	log_dbg("object_reached_termination_condition: vsm->get_angular_speed() = %g\n", vsm->get_angular_speed());
//	log_dbg("object_reached_termination_condition: vsm->get_linear_acceleration() = %g\n", vsm->get_linear_acceleration());
//	log_dbg("object_reached_termination_condition: vsm->get_angular_acceleration() = %g\n", vsm->get_angular_acceleration());

	if (object_visible_and_error_small && vsm->get_linear_speed() <= max_linear_speed && vsm->get_angular_speed()
			<= max_angular_speed && vsm->get_linear_acceleration() <= max_linear_accel
			&& vsm->get_angular_acceleration() <= max_angular_accel) {
		steps_delay++;
		if (steps_delay >= min_steps) {
			condition_met = true;
		}
	} else {
		steps_delay = 0;
	}
}

}//namespace generator

}

}
