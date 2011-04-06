/*
 * pb_sac_calibration.cc
 *
 *  Created on: 14-12-2010
 *      Author: mboryn
 */

#include <iostream>

#include "pb_sac_calibration.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace servovision {

using namespace std;
using namespace logger;

pb_sac_calibration::pb_sac_calibration(boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor, const std::string& section_name, mrrocpp::lib::configurator& configurator) :
	pb_visual_servo(boost::shared_ptr <visual_servo_regulator>(), sensor, section_name, configurator)
{
	logger::log_enabled = true;
	log_dbg("pb_sac_calibration::pb_sac_calibration() begin: section_name=%s\n", section_name.c_str());
	E_T_G = configurator.value <3, 4> ("E_T_G", section_name);
	log_dbg("pb_sac_calibration::pb_sac_calibration() end\n");
}

pb_sac_calibration::~pb_sac_calibration()
{
}

lib::Homog_matrix pb_sac_calibration::compute_position_change(const lib::Homog_matrix& current_position, double dt)
{
	lib::Homog_matrix C_T_G(reading.objectPosition.elements);

//	log_dbg("pb_sac_calibration::compute_position_change\n");

	lib::Homog_matrix O_T_C = current_position * E_T_G * (!C_T_G);

	all_O_T_C.push_back(O_T_C);

	lib::Homog_matrix delta_position;
	return delta_position;
}

std::vector<lib::Homog_matrix> pb_sac_calibration::get_all_O_T_C()
{
	return all_O_T_C;
}

} //namespace

}

}
