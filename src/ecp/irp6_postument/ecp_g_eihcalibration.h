/*
 * ecp_g_eihcalibration.h
 *
 *  Created on: July 28, 2009
 *      Author: jkosiore
 */

#ifndef ECP_G_EIHCALIBRATION_H_
#define ECP_G_EIHCALIBRATION_H_

//fradia
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace generator {

class eihgenerator : public common::generator::generator
{
/*	lib::ECP_VSP_MSG comm_struct;
	ecp_mp::sensor::cvfradia * sensor;
	lib::SENSOR_IMAGE calib_data;
*/

  public:
	eihgenerator(common::task::task& _ecp_task);
	~eihgenerator();
	bool first_step();
	bool next_step();

	void get_frame(double[12]);

};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_EIHCALIBRATION_H_ */

