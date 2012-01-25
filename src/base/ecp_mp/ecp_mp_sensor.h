/*!
 * @file
 * @brief File contains sensor class - a template base class for MP and ECP  sensors.
 * @author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup SENSORS
 */

#if !defined(_ECP_MP_SENSOR_H)
#define _ECP_MP_SENSOR_H

#include <string>

#include <boost/thread/thread.hpp>

#include <fcntl.h>

#include "ecp_mp_sensor_interface.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/lib/configurator.h"

#include "base/lib/messip/messip.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

const lib::sensor::SENSOR_t SENSOR_CAMERA_SA = "SENSOR_CAMERA_SA";
const lib::sensor::SENSOR_t SENSOR_CAMERA_ON_TRACK = "SENSOR_CAMERA_ON_TRACK";
const lib::sensor::SENSOR_t SENSOR_CAMERA_POSTUMENT = "SENSOR_CAMERA_POSTUMENT";

/**
 * @brief Base template class for all ECP and MP sensors.
 *
 * @tparam SENSOR_IMAGE .
 * @tparam CONFIGURE_DATA Structure sent to VSP along with the sensor configuration command
 *
 * @author tkornuta
 * @author ptrojane
 *
 * @ingroup SENSORS
 */
template <typename SENSOR_IMAGE, typename CONFIGURE_DATA = lib::empty_t>
class sensor : public ecp_mp::sensor::sensor_interface
{
private:
	/** @brief VSP process id. */
	pid_t pid;

	/** @brief Sensor descriptor. */
	lib::fd_client_t sd;

	/** @brief Sensor (VSP process) name. */
	std::string VSP_NAME;

	/** @brief Pointer to communication object. */
	lib::sr_ecp &sr_ecp_msg;

	/**
	 * @brief Buffer for data to VSP.
	 * @author ptrojane
	 * @author tkornuta
	 */
	struct VSP_ECP_MSG
	{
		/** @brief Report - status of the operation. */
		lib::sensor::VSP_REPORT_t vsp_report;

		/** @brief Aggregated reading - communication image. */
		SENSOR_IMAGE comm_image;

		//! Give access to boost::serialization framework
		friend class boost::serialization::access;

		//! Serialization of the data structure
		template <class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & vsp_report;
			ar & comm_image;
		}
	} from_vsp;

public:
	/** @brief Image of the sensor. */
	SENSOR_IMAGE image;

	/**
	 * @brief Buffer for data from VSP.
	 * @author ptrojane
	 * @author tkornuta
	 */
	struct ECP_VSP_MSG
	{
		/** @brief Command sent to VSP. */
		lib::sensor::VSP_COMMAND_t i_code;

		/** @brief Additional command parameters - FEATURE NOT IMPLEMENTED. */
		CONFIGURE_DATA command;

		//! Give access to boost::serialization framework
		friend class boost::serialization::access;

		//! Serialization of the data structure
		template <class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & i_code;
			ar & command;
		}
	} to_vsp;

	/** @brief Sensor name. */
	const lib::sensor::SENSOR_t sensor_name;

	/**
	 * @brief Base sensor constructor. Reads configuration, spawns a VSP process (if it doesn't already exists).
	 *
	 * @param _sensor_name Sensor name.
	 * @param _section_name Name of related section in configuration file.
	 * @param _sr_ecp_msg communication object.
	 * @param config Configuration object.
	 */
	sensor(lib::sensor::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config);

	/** @brief Virtual destructor. Sends TERMINATE command to VSP. */
	virtual ~sensor();

	/** @brief Sends configuration command to VSP. */
	virtual void configure_sensor(void);

	/** @brief Sends reading initialization command to VSP. */
	virtual void initiate_reading(void);

	/** @brief Retrieves reading from related VSP. */
	virtual void get_reading(void);

	/** @brief Overloaded version of reading retrieval - writes readings to output image.
	 *
	 * @param[out] sensor_image Output image.
	 */
	void get_reading(SENSOR_IMAGE & sensor_image);

};

template <typename SENSOR_IMAGE, typename CONFIGURE_DATA>
sensor <SENSOR_IMAGE, CONFIGURE_DATA>::sensor(lib::sensor::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config) :
		sr_ecp_msg(_sr_ecp_msg), sensor_name(_sensor_name)
{
	// Ustawienie domyslnego okresu pracy czujnika.
	base_period = current_period = 1;

	node_name = config.value <std::string>("node_name", _section_name);

	VSP_NAME = config.return_attach_point_name("resourceman_attach_point", _section_name);

	// cout<<"vsp_NAME = "<<VSP_NAME<<endl;

	// Sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow o tej nazwie.
	if ((sd = messip::port_connect(VSP_NAME))) {
		// by Y - usuniete bo mozna podlaczyc sie do istniejacego czujnika
		// throw sensor_error(lib::SYSTEM_ERROR, DEVICE_ALREADY_EXISTS);
		pid = 0; // tymczasowo
		return;
	}

	// Stworzenie nowego procesu.
	if ((pid = config.process_spawn(_section_name)) == -1) {
		BOOST_THROW_EXCEPTION(lib::exception::se_sensor() << lib::exception::mrrocpp_error0(CANNOT_SPAWN_VSP));
	}

	short tmp = 0;
	// Kilka sekund  (~2) na otworzenie urzadzenia.
	while ((sd = messip::port_connect(VSP_NAME)) == NULL) {
		// 		cout<<tmp<<endl;
		if ((tmp++) < lib::CONNECT_RETRY) {
			boost::this_thread::sleep(lib::CONNECT_DELAY);
		} else {
			std::cerr << "ecp_mp_sensor: messip::port_connect(" << VSP_NAME << ") failed" << std::endl;
			BOOST_THROW_EXCEPTION(lib::exception::se_sensor() << lib::exception::mrrocpp_error0(CANNOT_LOCATE_DEVICE));
		}
	} // end: while

}

template <typename SENSOR_IMAGE, typename CONFIGURE_DATA>
sensor <SENSOR_IMAGE, CONFIGURE_DATA>::~sensor()
{
	to_vsp.i_code = lib::sensor::VSP_TERMINATE;

	if (messip::port_send(sd, 0, 0, to_vsp, from_vsp) < 0)
		sr_ecp_msg.message(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);
	else
		messip::port_disconnect(sd);

	kill(pid, SIGTERM);

}

template <typename SENSOR_IMAGE, typename CONFIGURE_DATA>
void sensor <SENSOR_IMAGE, CONFIGURE_DATA>::configure_sensor(void)
{
	to_vsp.i_code = lib::sensor::VSP_CONFIGURE_SENSOR;

	if (messip::port_send(sd, 0, 0, to_vsp, from_vsp) < 0)

		sr_ecp_msg.message(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);
}

template <typename SENSOR_IMAGE, typename CONFIGURE_DATA>
void sensor <SENSOR_IMAGE, CONFIGURE_DATA>::initiate_reading(void)
{
	to_vsp.i_code = lib::sensor::VSP_INITIATE_READING;

	if (messip::port_send(sd, 0, 0, to_vsp, from_vsp) < 0)

		sr_ecp_msg.message(lib::SYSTEM_ERROR, CANNOT_WRITE_TO_DEVICE, VSP_NAME);
}

template <typename SENSOR_IMAGE, typename CONFIGURE_DATA>
void sensor <SENSOR_IMAGE, CONFIGURE_DATA>::get_reading(void)
{
	get_reading(image);
}

template <typename SENSOR_IMAGE, typename CONFIGURE_DATA>
void sensor <SENSOR_IMAGE, CONFIGURE_DATA>::get_reading(SENSOR_IMAGE & sensor_image)
{
	// Sprawdzenie, czy uzyc domyslnego obrazu.
	to_vsp.i_code = lib::sensor::VSP_GET_READING;

	if (messip::port_send(sd, 0, 0, to_vsp, from_vsp) < 0)

		sr_ecp_msg.message(lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME);

	// jesli odczyt sie powodl, przepisanie pol obrazu z bufora komunikacyjnego do image;
	if (from_vsp.vsp_report == lib::sensor::VSP_REPLY_OK) {
		image = from_vsp.comm_image;
	} else {
		sr_ecp_msg.message("Reply from VSP not ok");
	}
}

} // namespace sensor

} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_MP_SENSOR_H */
