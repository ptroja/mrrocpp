/*!
 * @file
 * @brief File containing declaration of the vsp sensor base template class.
 *
 * @date 09.11.2005
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 *
 * @ingroup VSP
 */

#if !defined(_VSP_SENSOR_H)
#define _VSP_SENSOR_H

#include "base/vsp/vsp_sensor_interface.h"

namespace mrrocpp {
namespace vsp {
namespace common {


/**
 * @brief Base template class, from which VSP kernels (sensors) should be derived.
 * @tparam VSP_ECP_MSG Response structure - sent from VSP to ECP.
 * @tparam ECP_VSP_MSG Command - structure sent from ECP to VSP, empty as default.
 *
 * @author ptrojane
 * @author tkornuta
 */
template <typename VSP_ECP_MSG, typename ECP_VSP_MSG = lib::empty_t>
class sensor : public sensor_interface
{
protected:

	/**
	 * @brief Message sent from VSP as response, containing operation status and its aggregated reading.
	 * @author ptrojane
	 * @author tkornuta
	 */
	struct from_vsp_t
	{
		/** @brief Report - status of the operation. */
		lib::sensor::VSP_REPORT_t vsp_report;

		/** @brief Aggregated reading - communication image. */
		VSP_ECP_MSG comm_image;
	} from_vsp;

	/**
	 * @brief Message received by VSP, containing command and additional parameters.
	 * @author ptrojane
	 * @author tkornuta
	 */
	struct to_vsp_t
	{
		/** \brief Command sent to VSP. */
		lib::sensor::VSP_COMMAND_t i_code;

		/** \brief Additional command parameters. */
		ECP_VSP_MSG to_vsp;
	} to_vsp;

public:

	/**
	 * @brief Default Constructor.
	 * @param _config Configuration object.
	 */
	sensor(lib::configurator &_config) :
		sensor_interface(_config)
	{
	}

	/**
	 * @brief Reads retrieved message from context to communication buffer.
	 * @param ctp Resource manager context.
	 * @return Status of the operation.
	 */
	int msgread(resmgr_context_t *ctp)
	{
		return resmgr_msgread(ctp, &to_vsp, sizeof(to_vsp), sizeof(struct _io_write));
	}

	/**
	 * @brief Writes reply message from communication buffer to context.
	 * @param ctp Resource manager context.
	 * @return Status of the operation.
	 */
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

	/**
	 * @brief Sets report returned by VSP.
	 * @param r Report to be set.
	 */
	void set_vsp_report(lib::sensor::VSP_REPORT_t r)
	{
		from_vsp.vsp_report = r;
	}

	/**
	 * @brief Returns command sent to VSP.
	 * @return Received command.
	 */
	lib::sensor::VSP_COMMAND_t get_command(void) const
	{
		return to_vsp.i_code;
	}

};


} // namespace common
} // namespace vsp
} // namespace mrrocpp

#endif
