/*!
 * \file ecp_mp_s_cvfradia.h
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * - class declaration
 * \author tkornuta
 * \date 15.03.2008
 */

#ifndef __ECP_CVFRADIA_H
#define __ECP_CVFRADIA_H

#include <netdb.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "ecp_mp/sensor/ecp_mp_sensor.h"
#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/*!
 * \brief Types commands sent to PW_HaarDetect task.
 */
typedef enum _HD_MODE
{
	WITHOUT_ROTATION, PERFORM_ROTATION
} hd_mode_t;

template<typename FROM_VSP_T, typename TO_VSP_T = lib::empty_t>
class cvfradia : public fradia_sensor<FROM_VSP_T, TO_VSP_T> {
public:
	cvfradia (lib::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config)
			: fradia_sensor<FROM_VSP_T, TO_VSP_T>(config, _section_name)
	{
	}
};

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
