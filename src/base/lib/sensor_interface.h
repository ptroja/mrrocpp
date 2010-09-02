/*!
 * @file sensor_interface.h
 * @brief File contains sensor interface - a base class for MP, ECP, VSP (and future EDP ) sensors.
 * @author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup SENSORS lib
 */

#if !defined(_SENSOR_H)
#define _SENSOR_H

#include <string>

#include "base/lib/com_buf.h"
#include "sensor_error.h"

namespace mrrocpp {
namespace lib {
namespace sensor {

/**
 * @brief Commands send to VSP.
 * @author tkornuta
 *
 * @ingroup SENSORS
 */
typedef enum _VSP_COMMAND
{
	VSP_CONFIGURE_SENSOR, VSP_INITIATE_READING, VSP_GET_READING, VSP_TERMINATE
} VSP_COMMAND_t;

/**
 * @brief VSP responses.
 * @author tkornuta
 *
 * @ingroup SENSORS
 */
typedef enum _VSP_REPORT
{
	VSP_REPLY_OK, VSP_SENSOR_NOT_CONFIGURED, VSP_READING_NOT_READY, INVALID_VSP_COMMAND
} VSP_REPORT_t;

/**
 * @brief Sensor names type.
 *
 * @ingroup SENSORS
 */
typedef std::string SENSOR_t;

/**
 * @brief Base class for MP, ECP, VSP (and future EDP ) sensors.
 * @author tkornuta
 * @author ptrojane
 *
 * @ingroup SENSORS
 */
class sensor_interface
{
public:
	/**
	 * Abstract method responsible for reading retrieval.
	 */
	virtual void get_reading(void) = 0;

	/**
	 * Virtual method responsible for sensor configuration.
	 */
	virtual void configure_sensor(void)
	{
	}

	/**
	 * Virtual method responsible for reading initialization.
	 */
	virtual void initiate_reading(void)
	{
	}

	/**
	 * Virtual destructor. Empty.
	 */
	virtual ~sensor_interface()
	{
	}
};

} // namespace sensor
} // namespace lib
} // namespace mrrocpp

#endif /* _SENSOR_H */
