/*
 * EcpMpAndroid.cpp
 *
 *  Created on: Nov 20, 2011
 *      Author: hh7
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <strings.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

#include "application/android_teach/sensor/EcpMpAndroid.h"
//#include "base/ecp_mp/ecp_mp_task.h"

using namespace std;

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

EcpMpAndroid::EcpMpAndroid(lib::sensor::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config, android_teach::AndroidState* androidState, ecp::common::generator::get_position* gg)
	: sr_ecp_msg(_sr_ecp_msg), sensor_name(_sensor_name), androidState(androidState), gg(gg)
{
	// Set period variables.
	base_period=current_period=1;


	// Retrieve wiimote node name and port from configuration file.
	int android_port = config.value<int>("android_port", _section_name);
	std::string android_node_name = config.value<std::string>("android_node_name", _section_name);


    // Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		sr_ecp_msg.message("ERROR opening socket");
		throw android_teach::NetworkException();
	}


	// Get server hostname.
	server = gethostbyname(android_node_name.c_str());
	if (server == NULL) {
		//buffer used for communication
		char buffer[128];

		sprintf(buffer,"ERROR, no host %s", android_node_name.c_str());
		sr_ecp_msg.message(buffer);
		throw android_teach::NetworkException();
	}


	// Reset socketaddr data.
	bzero((char *) &serv_addr, sizeof(serv_addr));
	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(android_port);



//	error = boost::asio::error::host_not_found;
//	boost::asio::ip::tcp::resolver::query query(serv_addr,"40666");
//	endpoint_iter = resolver.resolve(query);


	replyMsgPtr = (android_teach::VspEcpMsg*)replyBuffer;
	replyErrorMsgPtr = (android_teach::VspEcpErrorMsg*)replyBuffer;
	replyReadingsMsgPtr = (android_teach::VspEcpReadingsMsg*)replyBuffer;

}


EcpMpAndroid::~EcpMpAndroid()
{
	if (androidState->connected)
		disconnect();
}

bool EcpMpAndroid::connectToServer()
{
	// Try to establish a connection with android.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		sr_ecp_msg.message("Error connecting");
		throw android_teach::NetworkException();
	}


	sr_ecp_msg.message("Connected to android2");
	androidState->connected = true;

	return androidState->connected;
}




void EcpMpAndroid::configure_sensor()
{
//	// Send adequate command to wiimote.
//	android_teach::EcpVspConfigMsg to_vsp;
//	if(write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)
//		throw android_teach::NetworkException();
}


void EcpMpAndroid::initiate_reading() {
//	// Send adequate command to wiimote.
////	to_vsp.i_code = lib::sensor::VSP_INITIATE_READING;
//	android_teach::EcpVspMsg to_vsp(android_teach::ECP_VSP_START);
//	if(write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)
//		throw android_teach::NetworkException();
//
//	//read
//	android_teach::VspEcpMsg from_vsp;
//	if(read(sockfd, &from_vsp, sizeof(from_vsp))==-1)
//		throw android_teach::NetworkException();
//
// 	if (from_vsp.msgType != android_teach::VSP_ECP_CONFIRM)
// 		sr_ecp_msg.message ("Reply from VSP not ok");

}

bool EcpMpAndroid::isAbort() {
	// Send adequate command to wiimote.

	stateMsg.msgType = android_teach::ECP_VSP_WAIT;

//	androidState->updateCurrentPosition(gg->get_position_vector());
	stateMsg.ecpStatus = androidState->ecpStatus;

	if(write(sockfd, &stateMsg, sizeof(stateMsg)) == -1)
		throw android_teach::NetworkException();

	//read

	if(read(sockfd, &replyBuffer, sizeof(replyBuffer))==-1)
		throw android_teach::NetworkException();

 	if (replyMsgPtr->msgType == android_teach::VSP_ECP_WAIT)
 		return false;
 	else if (replyMsgPtr->msgType == android_teach::VSP_ECP_ABORT)
 		return true;
 	else if (replyMsgPtr->msgType == android_teach::VSP_ECP_DISCONNECT)
 	{
 		setDisconnectState();
 		return true;
 	}
 	else
 	{
 		sr_ecp_msg.message ("Reply from VSP in isAbort() not ok");
 		return true;
 	}
}

bool EcpMpAndroid::sendStart()
{
	perror("*********************************SENDSTART*************");
	msg.msgType = android_teach::ECP_VSP_START;


	if(write(sockfd, &msg, sizeof(msg)) == -1)
		throw android_teach::NetworkException();


	//read
	len = read(sockfd, &replyBuffer, sizeof(replyBuffer));

	if(	len ==-1)
		throw android_teach::NetworkException();
	else
	{
		perror("poprawny odczyt");
	}
	char plebuf[100];
	sprintf(plebuf,"sendstart reply from vsp: %d, przeczytano: %d", replyMsgPtr->msgType, len);
	perror(plebuf);


 	if (replyMsgPtr->msgType == android_teach::VSP_ECP_WAIT_BEFORE_START)
 	{
 		perror("send Start reply wait_before_start");
 		return false;
 	}
 	else if (replyMsgPtr->msgType == android_teach::VSP_ECP_CONFIRM)
 	{
 		perror("send Start reply confirm");
 		return true;
 	}
 	else if (replyMsgPtr->msgType == android_teach::VSP_ECP_DISCONNECT)
 	{
 		perror("send Start reply disconnect");
 		setDisconnectState();
 		return true;
 	}
 	else
 	{
 		perror("send Start reply unknown");
 		sr_ecp_msg.message ("Reply from VSP in sendStart() not ok");
 		return false;
 	}
}

bool EcpMpAndroid::sendHello()
{
	msg.msgType = android_teach::ECP_VSP_HELLO;


	if(write(sockfd, &msg, sizeof(msg)) == -1)
		throw android_teach::NetworkException();


	//read
	len = read(sockfd, &replyBuffer, sizeof(replyBuffer));

	if(	len ==-1)
		throw android_teach::NetworkException();
	else
	{
		perror("poprawny odczyt");
	}
	char plebuf[100];
	sprintf(plebuf,"sendHello reply from vsp: %d, przeczytano: %d", replyMsgPtr->msgType, len);
	perror(plebuf);


 	if (replyMsgPtr->msgType == android_teach::VSP_ECP_HELLO)
 	{
 		perror("send Start reply hello");
 		return false;
 	}
 	else if (replyMsgPtr->msgType == android_teach::VSP_ECP_DISCONNECT)
 	{
 		perror("send Hello reply disconnect");
 		setDisconnectState();
 		return true;
 	}
 	else
 	{
 		perror("send Hello reply unknown");
 		sr_ecp_msg.message ("Reply from VSP in sendHello() not ok");
 		return false;
 	}
}


bool EcpMpAndroid::sendConfiguration()
{

	android_teach::EcpVspConfigMsg to_vsp(android_teach::ECP_VSP_CONFIGURE);
	to_vsp.config = androidState->config;


	if(write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)
		throw android_teach::NetworkException();

	//read
	if(read(sockfd, &replyBuffer, sizeof(replyBuffer))==-1)
		throw android_teach::NetworkException();



 	if (replyMsgPtr->msgType == android_teach::VSP_ECP_CONFIRM)
 		return true;
 	else if (replyMsgPtr->msgType == android_teach::VSP_ECP_DISCONNECT)
 	{
 		setDisconnectState();
 		return true;
 	}
 	else
 	{
 		sr_ecp_msg.message ("Reply from VSP in sendCOnfiguration() not ok");
 		return false;
 	}


}


void EcpMpAndroid::send_reading() {
//	// Send any command to wiimote.
//	android_teach::EcpVspMsg to_vsp;
//	if(write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)
//		throw android_teach::NetworkException();
}


void EcpMpAndroid::get_reading() {
	//Send to android


	get_reading(1);
}

void EcpMpAndroid::get_reading(int mode) {
//	// Send adequate command to wiimote.
//
//	android_teach::EcpVspMsg to_vsp(android_teach::ECP_VSP_HELLO);
//	android_teach::VspEcpMsg from_vsp;
//
//	if(write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)
//		throw android_teach::NetworkException();
//
//
//	// Read aggregated data from wiimote.
// 	if(read(sockfd, &from_vsp, sizeof(from_vsp))==-1)
//		throw android_teach::NetworkException();
//
//	// Check and copy data from buffer to image.
////	if(from_vsp.vsp_report == lib::sensor::VSP_REPLY_OK)
////		image = from_vsp.wiimote;
////	else
////		sr_ecp_msg.message ("Reply from VSP not ok");
// 	if (from_vsp.msgType != android_teach::VSP_ECP_HELLO)
// 		sr_ecp_msg.message ("Reply from VSP not ok");
}



void EcpMpAndroid::getReadings()
{
	getReadings(android_teach::ECP_VSP_GET_READING);
}


void EcpMpAndroid::getReadings(android_teach::ECP_VSP_MSG_TYPE msgType)
{
	stateMsg.msgType = msgType;


	//androidState->updateCurrentPosition(gg->get_position_vector());
	stateMsg.ecpStatus = androidState->ecpStatus;

	if(write(sockfd, &stateMsg, sizeof(stateMsg)) == -1)
		throw android_teach::NetworkException();


	// Read data from android.
 	if(read(sockfd, &replyBuffer, sizeof(replyBuffer))==-1)
		throw android_teach::NetworkException();


 	if (replyMsgPtr->msgType == android_teach::VSP_ECP_READINGS)
 	{
 		androidState->readings = replyReadingsMsgPtr->readings;
 	}
 	else if (replyMsgPtr->msgType == android_teach::VSP_ECP_DISCONNECT)
 	{
 		setDisconnectState();
 	}
 	else
 	{
 		sr_ecp_msg.message ("Reply from VSP in getReadings not ok");
 	}

 	if (androidState->readings.mode == 2)
 	{
 		perror("getReadings() read mode = 2");
 	}
 	else if (androidState->readings.mode == 1)
 	{
 		perror("getReadings() read mode = 1");
 	}


}


void EcpMpAndroid::sendEndOfMotion()
{

	stateMsg.msgType = android_teach::ECP_VSP_RESUME;
	androidState->updateCurrentPosition(gg->get_position_vector());
	stateMsg.ecpStatus = androidState->ecpStatus;


	if(write(sockfd, &stateMsg, sizeof(stateMsg)) == -1)
		throw android_teach::NetworkException();


 	if(read(sockfd, &replyBuffer, sizeof(replyBuffer))==-1)
		throw android_teach::NetworkException();


 	perror("after send and read in endOfMotion()");

 	if (replyMsgPtr->msgType == android_teach::VSP_ECP_CONFIRM)
 	{
 		//nothing to write
 	}
 	else if (replyMsgPtr->msgType == android_teach::VSP_ECP_DISCONNECT)
 	{
 		setDisconnectState();
 	}
 	else
 	{
 		sr_ecp_msg.message ("Reply from VSP in sendEndOfMotion not ok");
 		return;
 	}
}


void EcpMpAndroid::disconnect()
{
	msg.msgType = android_teach::ECP_VSP_DISCONNECT;
	perror("*********************************DISCONNECT*************");


	if(write(sockfd, &msg, sizeof(msg)) == -1)
		throw android_teach::NetworkException();


// 	if(read(sockfd, &replyBuffer, sizeof(replyBuffer))==-1)
//		throw android_teach::NetworkException();
//
//	char plebuf[100];
//	sprintf(plebuf,"disconnect reply from vsp: %d, przeczytano: %d", (int)replyBuffer[0], len);
//	perror(plebuf);
//
//
// 	if (replyMsgPtr->msgType == android_teach::VSP_ECP_DISCONNECT)
// 	{
 		//nothing to write
		setDisconnectState();
// 	}
// 	else
// 	{
// 		sr_ecp_msg.message ("Reply from VSP in disconnect not ok");
// 		return;
// 	}
}

void EcpMpAndroid::setDisconnectState()
{
	androidState->readings.mode = 3;
	androidState->connected = false;
	close(sockfd);
	sr_ecp_msg.message("Disconnected\n");
	sr_ecp_msg.message("Terminate\n");
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
