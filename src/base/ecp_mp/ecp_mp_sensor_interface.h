/*!
 * @file
 * @brief File contains sensor interface for all MP and ECP  sensors.
 * 
 * @author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @date Aug 4, 2010
 *
 * @ingroup SENSORS
 */


#ifndef ECP_MP_SENSOR_INTERFACE_H_
#define ECP_MP_SENSOR_INTERFACE_H_

#include "base/lib/sensor_interface.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {


/**
 * @brief Interface class for all ECP and MP sensors.
 * @author ptrojane
 * @author tkornuta
 *
 * @ingroup SENSORS
 */
class sensor_interface : public lib::sensor::sensor_interface
{
public:
	/**
	 * @brief Variable defining by which generator step a new reading from the VSP should be pulled (a sequence of initialize and get reading).
	 *
	 * In particular it is possible to avoid reading initialization and retrieval by setting it to 0.
	 */
	short base_period;

	/**
	 * @brief Number of steps left to next reading initialization and retrieval.
	 */
	short current_period;

	/** @brief VSP node name. */
	std::string node_name;
};


} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp


#endif /* ECP_MP_SENSOR_INTERFACE_H_ */
