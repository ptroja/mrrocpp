/*
 * ECP_G_ROBOT_CALIBRATION.h
 *
 *  Created on: May 08, 2009
 *      Author: mani.baktha
 */

#ifndef ECP_G_ROBOT_CALIBRATION_H_
#define ECP_G_ROBOT_CALIBRATION_H_

//PCBIRD Virtual Sensors defined here
#include "ecp_mp/sensor/ecp_mp_s_pcbird.h"
#include "ecp/common/ecp_t_pcbird.h"

// we will write a generator robocalibgen derived from generator namespace 
#include "ecp/common/ecp_generator.h"	


namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

// robocalibgen similar to spots.class representing generator
class robotcalibgen : public common::generator::generator
{
	short iter;
	ecp_mp::sensor::pcbird * sensor;
	// sensor image to receive the readings from pcbird
	lib::SENSOR_IMAGE pcbird_sensor_image;


  public:
	robotcalibgen(common::task::task& _ecp_task);
	bool first_step();
	bool next_step();

};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_ROBOT_CALIBRATION_H */
