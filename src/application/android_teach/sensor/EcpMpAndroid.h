/*
 * EcpMpAndroid.h
 *
 *  Created on: Nov 20, 2011
 *      Author: hh7
 */

#ifndef ECPMPANDROID_H_
#define ECPMPANDROID_H_




#include "generator/ecp/get_position/ecp_g_get_position.h"
#include <netdb.h>
#include <netinet/in.h>
#include "base/ecp_mp/ecp_mp_sensor.h"
#include "application/android_teach/Enums.h"
#include "application/android_teach/AndroidState.h"
#include "application/android_teach/NetworkException.h"



namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

#define REPLY_BUFFER_SIZE 512
#define MSG_BUFFER_SIZE 512

/** @addtogroup wii_teach
 *
 *  @{
 */




const lib::sensor::SENSOR_t SENSOR_ANDROID = "SENSOR_ANDROID";

/**
 * Virtual sensor that communicates with the Wii-mote
 *
 * @author jedrzej
 */
class EcpMpAndroid : public sensor_interface
{

private:
	//socket descriptor
	int sockfd;

	//address of the wii-mote server
	sockaddr_in serv_addr;

	//pointer to the server
	hostent* server;

	//link to SRP communication object
	lib::sr_ecp& sr_ecp_msg;

	char msgBuffer[MSG_BUFFER_SIZE];
	char replyBuffer[REPLY_BUFFER_SIZE];

	//sensor name
	const lib::sensor::SENSOR_t sensor_name;

	const android_teach::VspEcpMsg* replyMsgPtr;
    const android_teach::VspEcpErrorMsg* replyErrorMsgPtr;
    const android_teach::VspEcpReadingsMsg* replyReadingsMsgPtr;

    android_teach::EcpVspMsg msg;
    android_teach::EcpVspStateMsg stateMsg;

    size_t replySize;
    size_t len;

    android_teach::AndroidState* androidState;

    ecp::common::generator::get_position* gg;



public:

	/**
	 * Sends sensor configuration to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void configure_sensor (void);

	/**
	 * Sends "initiate reading" command to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void initiate_reading (void);

	/**
	 * Sends given command to the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void send_reading ();

	/**
	 * Retrieves aggregated data from the Wii-mote server
	 *
	 * @author jedrzej
	 */
	void get_reading (void);

	void get_reading (int);



	/**
	 * Creates the sensor object. Connects to the Wii-mote server
	 *
	 * @param _sensor_name 		the name f the sensor
	 * @param _section_name		the name of the section in the config file
	 * @param _ecp_mp_object
	 * @param _union_size
	 *
	 * @author OL
	 */
	EcpMpAndroid(lib::sensor::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config, android_teach::AndroidState* androidState, ecp::common::generator::get_position* gg);



	/**
	 * Terminates the connection to the Wii-mote server
	 *
	 * @author OL
	 */

	~EcpMpAndroid();

	bool connectToServer();
	bool sendStart();
	void getReadings(android_teach::ECP_VSP_MSG_TYPE msgType);
	void getReadings();
	void disconnect();
	void setDisconnectState();
	void sendErrorCode(int errorCode);
	bool sendConfiguration();
	void sendEndOfMotion();
	bool isAbort(void);
	bool sendHello();

};


} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* ECPMPANDROID_H_ */
