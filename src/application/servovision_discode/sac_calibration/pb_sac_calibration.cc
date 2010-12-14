/*
 * pb_sac_calibration.cc
 *
 *  Created on: 14-12-2010
 *      Author: mboryn
 */

#include <iostream>

#include "pb_sac_calibration.h"
#include "../visual_servo_regulator_p.h"

#include "base/lib/logger.h"


namespace mrrocpp {

namespace ecp {

namespace servovision {

using namespace std;
using namespace logger;

pb_sac_calibration::pb_sac_calibration(boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor, const std::string& section_name, mrrocpp::lib::configurator& configurator) :
	pb_visual_servo(boost::shared_ptr <visual_servo_regulator>(new regulator_p(configurator, section_name)), sensor, section_name, configurator)
{
	log_dbg("pb_sac_calibration::pb_sac_calibration() begin");
	E_T_G = configurator.value <3, 4> ("E_T_G", section_name);
	log_dbg("pb_sac_calibration::pb_sac_calibration() end");
}

pb_sac_calibration::~pb_sac_calibration()
{
}

lib::Homog_matrix pb_sac_calibration::compute_position_change(const lib::Homog_matrix& current_position, double dt)
{
	lib::Homog_matrix C_T_G(reading.objectPosition.elements);

	lib::Homog_matrix O_T_C = current_position * E_T_G * (!C_T_G);

	cout<<"=====================================================================\n";
	cout<<"\n\nO_T_C = \n" << O_T_C << "\n";
	cout<<"=====================================================================\n";
	cout.flush();

	lib::Homog_matrix delta_position;
	return delta_position;
}

} //namespace

}

}
