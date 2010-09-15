/*!
 * @file
 * @brief File containing declaration of the base sensor_interface class.
 *
 * @date 04.09.2010
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 *
 * @ingroup VSP
 */

#if !defined(_VSP_SENSOR_INTERFACE_H)
#define _VSP_SENSOR_INTERFACE_H

#include "base/lib/sensor_interface.h"
#include "base/lib/configurator.h"
#include "base/lib/sr/sr_vsp.h"

namespace mrrocpp {
namespace vsp {
namespace common {

/**
 * @brief Structure used during the communication with the VSP via DEVCTL.
 */
typedef struct DEVCTL_MSG_t
{
	/** Block of memory for message.  */
	char foo[2048];
} DEVCTL_MSG;


/**
 * @def DEVCTL_RD
 * @brief Macro used for shortening the call to message data during the read operation.
 * @author tkornuta
 */
#define DEVCTL_RD __DIOF(_DCMD_MISC, 1, mrrocpp::vsp::common::DEVCTL_MSG)

/**
 * @def DEVCTL_WT
 * @brief Macro used for shortening the call to message data during the write operation.
 * @author tkornuta
 */
#define DEVCTL_WT __DIOT(_DCMD_MISC, 2, mrrocpp::vsp::common::DEVCTL_MSG)

/**
 * @def DEVCTL_RW
 * @brief Macro used for shortening the call to message data during the read-write operation.
 * @author tkornuta
 */
#define DEVCTL_RW __DIOTF(_DCMD_MISC, 3, mrrocpp::vsp::common::DEVCTL_MSG)



/**
 * @brief Interface class for all sensors that are used in the Resource Manager based shells.
 * @author ptrojane
 * @author tkornuta
 */
class sensor_interface : public lib::sensor::sensor_interface
{
protected:
	/** @brief Flag used for determination whether sensor is configured. */
	bool is_sensor_configured;

	/** @brief Flag used for determination whether reading is ready. */
	bool is_reading_ready;

public:
	/** @brief Configuration object. */
	lib::configurator &config;

	/** @brief Object used for communication with the SR thread of the UI. */
	lib::sr_vsp *sr_msg;

	/** @brief Network path to the MRROC++ binaries. */
	const std::string mrrocpp_network_path;

	/**
	 * @brief Constructor.
	 * @param _config Configuration object.
	 * @return Newly created sensor object.
	 */
	sensor_interface(lib::configurator &_config);

	/**
	 * @brief Destructor.
	 */
	virtual ~sensor_interface(void);

	/**
	 * @brief Abstract method for setting the results of the sensor data aggregation.
	 * @param _report Result that is to be set.
	 */
	virtual void set_vsp_report(lib::sensor::VSP_REPORT_t _report) = 0;

	/**
	 * @brief Abstract method returning command sent to VSP.
	 * @return Retrieved command.
	 */
	virtual lib::sensor::VSP_COMMAND_t get_command(void) const = 0;

	/**
	 * @brief Method used by the non-interactive VSP shell - used as a trigger for next data processing start.
	 */
	virtual void wait_for_event(void);

	/**
	 * @brief Abstract method used for reading the message (retrieved command) from the Resource Manager context.
	 * @param ctp Resource Manager context.
	 * @return Operation status.
	 */
	virtual int msgread(resmgr_context_t *ctp) = 0;

	/**
	 * @brief Abstract method used for writing the message (newest reading) to the Resource Manager context.
	 * @param ctp Resource Manager context.
	 * @return Operation status.
	 */
	virtual int msgwrite(resmgr_context_t *ctp) = 0;

};


/**
 * @brief Function returns created sensor - implemented once in every VSP.
 * Users should use the  \link mrrocpp::vsp::common::VSP_REGISTER_SENSOR VSP_REGISTER_SENSOR\endlink macro, which shortens the definition.
 * @param _config Configurator object passed to sensor
 * @return Created sensor inherited from the sensor_interface class
 */
sensor_interface * return_created_sensor(lib::configurator &_config);


/**
 * @def VSP_REGISTER_SENSOR
 * @brief Macro used for sensor registration - should be used once for each VSP process.
 */
#define VSP_REGISTER_SENSOR(NAME) \
sensor_interface * mrrocpp::vsp::common::return_created_sensor (lib::configurator &_config) \
{ \
	return new NAME(_config); \
}

} // namespace common
} // namespace vsp
} // namespace mrrocpp

#endif
