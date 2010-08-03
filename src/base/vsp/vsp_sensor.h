/*!
 * @file vsp_sensor.h
 * @brief File containing declaration of the base sensor_interface class.
 *
 * @date 09.11.2005
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 *
 * @ingroup VSP
 */

#if !defined(_VSP_SENSOR_H)
#define _VSP_SENSOR_H

#include "lib/sensor.h"
#include "lib/configurator.h"

namespace mrrocpp {
namespace vsp {
namespace common {

/**
 * @brief Structure used during the communication with the VSP via DEVCTL.
 */
typedef struct
{
	/** Block of memory for message.  */
	char foo[2048];
} DEVCTL_MSG;


/**
 * @def DEVCTL_RD __DIOF
 * @brief Macro used for shortening the call to message data during the read operation.
 * @author tkornuta
 */
#define DEVCTL_RD __DIOF(_DCMD_MISC, 1, mrrocpp::vsp::common::DEVCTL_MSG)

/**
 * @def DEVCTL_RD __DIOF
 * @brief Macro used for shortening the call to message data during the write operation.
 * @author tkornuta
 */
#define DEVCTL_WT __DIOT(_DCMD_MISC, 2, mrrocpp::vsp::common::DEVCTL_MSG)

/**
 * @def DEVCTL_RD __DIOF
 * @brief Macro used for shortening the call to message data during the read-write operation.
 * @author tkornuta
 */
#define DEVCTL_RW __DIOTF(_DCMD_MISC, 3, mrrocpp::vsp::common::DEVCTL_MSG)



/**
 * @brief Interface class for all sensors that are used in the Resource Manager based shells.
 * @author ptrojane
 * @author tkornuta
 */
class sensor_interface : public lib::sensor_interface
{
protected:
	/** Flag used for determination whether sensor is configured. */
	bool is_sensor_configured;

	/** Flag used for determination whether reading is ready. */
	bool is_reading_ready;

public:
	/** Configuration object. */
	lib::configurator &config;

	/** Object used for communication with the SR thread of the UI. */
	lib::sr_vsp *sr_msg;

	/** Network path to the MRROC++ binaries. */
	const std::string mrrocpp_network_path;

	/**
	 * Constructor.
	 * @param _config Configuration object.
	 * @return Newly created sensor object.
	 */
	sensor_interface(lib::configurator &_config);

	/**
	 * Destructor.
	 */
	virtual ~sensor_interface(void);

	/**
	 * Abstract method for setting the results of the sensor data aggregation.
	 * @param  Result that is to be set.
	 */
	virtual void set_vsp_report(lib::VSP_REPORT_t) = 0;

	/**
	 * Abstract method returning command sent to VSP.
	 * @return Retrieved command.
	 */
	virtual lib::VSP_COMMAND_t get_command(void) const = 0;

	/**
	 * Method used by the non-interactive VSP shell - used as a trigger for next data processing start.
	 */
	virtual void wait_for_event(void);

	/**
	 * Abstract method used for reading the message (retrieved command) from the Resource Manager context.
	 * @param ctp Resource Manager context.
	 * @return Operation status.
	 */
	virtual int msgread(resmgr_context_t *ctp) = 0;

	/**
	 * Abstract method used for writing the message (newest reading) to the Resource Manager context.
	 * @param ctp Resource Manager context.
	 * @return Operation status.
	 */
	virtual int msgwrite(resmgr_context_t *ctp) = 0;

};

/**
 *
 */
template <typename VSP_ECP_MSG, typename ECP_VSP_MSG = lib::empty_t>
class sensor : public sensor_interface
{
protected:
	struct
	{
		lib::VSP_REPORT_t vsp_report;
		VSP_ECP_MSG comm_image;
	} from_vsp;

	struct
	{
		lib::VSP_COMMAND_t i_code;
		ECP_VSP_MSG to_vsp;
	} to_vsp;
public:
	sensor(lib::configurator &_config) :
		sensor_interface(_config)
	{
	}

	int msgread(resmgr_context_t *ctp)
	{
		return resmgr_msgread(ctp, &to_vsp, sizeof(to_vsp), sizeof(struct _io_write));
	}

	int msgwrite(resmgr_context_t *ctp)
	{
		// Compute  the start address of reply message content.
		/*
		 struct _io_devctl_reply {
		 uint32_t                  zero;
		 int32_t                   ret_val;
		 int32_t                   nbytes;
		 int32_t                   zero2;
		 // char                      data[nbytes];//
		 =>
		 &data = &_io_devctl_reply + 16bytes = &_io_devctl_reply + 4*int
		 */
		return resmgr_msgwrite(ctp, &from_vsp, sizeof(from_vsp), 0);
	}

	void set_vsp_report(lib::VSP_REPORT_t r)
	{
		from_vsp.vsp_report = r;
	}

	lib::VSP_COMMAND_t get_command(void) const
	{
		return to_vsp.i_code;
	}
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
