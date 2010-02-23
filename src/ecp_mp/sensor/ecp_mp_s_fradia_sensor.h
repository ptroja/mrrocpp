/*!
 * \file ecp_mp_s_cvfradia.h
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * - class declaration
 * \author tkornuta
 * \date 15.03.2008
 */

#ifndef __FRADIA_SENSOR_BLOCKING_H
#define __FRADIA_SENSOR_BLOCKING_H

#include <netdb.h>
#include <netinet/in.h>
#include "ecp_mp/sensor/ecp_mp_sensor.h"
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

#define BUFFER_SIZE 8*256

/*!
 * \class cvfradia
 * \brief Virtual sensor on the ECP/MP side used for communication with cvFraDIA framework.
 * \author tkornuta
 */
class fradia_sensor : public lib::sensor
{
public:

	/*!
      * Constructor. Creates socket connection to cvFraDIA.
      */
	fradia_sensor (lib::SENSOR_t _sensor_name, const char* _section_name, task::task& _ecp_mp_object);

	/*!
      * Sends sensor configuration to cvFraDIA.
      */
	void configure_sensor (void);

	/*!
      * Sends initiation reading command to cvFraDIA.
      */
	void initiate_reading (void);

	/*!
      * Sends given reading command to cvFraDIA.
      */
	void send_reading (lib::ECP_VSP_MSG);

	/*!
      * Retrieves aggregated data from cvFraDIA.
      */
	void get_reading (void);

	/*!
      * Closes cvFraDIA socket connection.
      */
	~fradia_sensor();

	void operator()();
private:
	/*!
      * Socket file descriptor.
      */
	int sockfd;

	/*!
      * Server address.
      */
	sockaddr_in serv_addr;

	/*!
      * Pointer to server.
      */
	hostent* server;

	/*!
      * Buffer used by sockets during communication.
      */
	char buffer[BUFFER_SIZE];

	/*!
      * Link to the SRP communication object.
      */
	lib::sr_ecp& sr_ecp_msg;

	/*!
      * Sensor name.
      */
	const lib::SENSOR_t sensor_name;

	lib::ECP_VSP_MSG to_vsp_tmp;

	lib::VSP_ECP_MSG from_vsp_tmp;
	/*!
	 * Thread which continously read measurments from fradia over TCP/IP socket.
	 * Measurements are buffered and always available to get_reading().
	 */
	boost::thread* reader_thread;

	/**
	 * Mutex for from_vsp_tmp.
	 */
	boost::mutex get_reading_mutex;


};

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
