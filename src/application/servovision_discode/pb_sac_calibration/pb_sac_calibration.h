/*
 * pb_sac_calibration.h
 *
 *  Created on: 14-12-2010
 *      Author: mboryn
 */

#ifndef PB_SAC_CALIBRATION_H_
#define PB_SAC_CALIBRATION_H_

#include "../pb_visual_servo.h"

namespace mrrocpp {

namespace ecp {

namespace servovision {


class pb_sac_calibration : public mrrocpp::ecp::servovision::pb_visual_servo
{
public:
	pb_sac_calibration(boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor, const std::string& section_name, mrrocpp::lib::configurator& configurator);
	virtual ~pb_sac_calibration();
protected:
	virtual lib::Homog_matrix compute_position_change(const lib::Homog_matrix& current_position, double dt);

	lib::Homog_matrix E_T_G;
};

} //namespace

}

}


#endif /* PB_SAC_CALIBRATION_H_ */
