/*
 * neuron_sensor.cpp
 *
 *  Created on: Jun 23, 2010
 *      Author: tbem
 */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "neuron_sensor.h"

#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <netinet/in.h>

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/*Sensor creation along with initialization of communication*/
neuron_sensor::neuron_sensor(mrrocpp::lib::configurator& _configurator):config(_configurator) {

	base_period=5;
	current_period=1;

	uint16_t vsp_port = config.value<uint16_t>("vsp_port", "[VSP]");
	const std::string vsp_node_name = config.value<std::string>("vsp_node_name", "[VSP]");

	printf("%d %s\n", vsp_port, vsp_node_name.c_str());

	//Try to open socket.
	socketDescriptor=socket(AF_INET, SOCK_STREAM, 0);
	if(socketDescriptor==-1)
		throw std::runtime_error("socket(): "+std::string(strerror(errno)));

	//Set socket options.
	int flag=1;
	if(setsockopt(socketDescriptor,IPPROTO_TCP,TCP_NODELAY,(char*) &flag, sizeof(int))==-1)
		throw std::runtime_error("setsockopt(): "+std::string(strerror(errno)));

	//Get server hostname.
	hostent* server=gethostbyname(vsp_node_name.c_str());
	if(server==NULL)
		throw std::runtime_error("gethostbyname("+vsp_node_name+"): "+std::string(hstrerror(h_errno)));

	//Data with addres of connection.
	sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));

	//Fill it with data.
	serv_addr.sin_family=AF_INET;
	memcpy(&serv_addr.sin_addr.s_addr, server->h_addr,server->h_length);
	serv_addr.sin_port=htons(vsp_port);

	//Try to estabilish a connection with neuron VSP.
	if(connect(socketDescriptor, (const struct sockaddr*) &serv_addr, sizeof(serv_addr))==-1)
		throw std::runtime_error("connect(): "+std::string(strerror(errno)));

	printf("Neuron sensor created\n");
}

neuron_sensor::~neuron_sensor() {
	close(socketDescriptor);
}

void neuron_sensor::get_reading(){
	printf("getReading\n");
	char buff[26];

	//Read packet from socket*/
	int result = read(socketDescriptor, buff, sizeof(buff));
	printf("size of buff %d %d\n",sizeof(buff),sizeof(double));
	if (result < 0) {
		throw std::runtime_error(std::string("read() failed: ") + strerror(errno));
	}

	//check whether whole incoming packet received
	if (result != sizeof(buff)) {
		throw std::runtime_error("read() failed: result != sizeof(MESSAGE_T)");
	}

	//copy data from packet to variables
	memcpy(&command,buff,2);
	memcpy(&(coordinates.x),buff+2,8);
	memcpy(&(coordinates.y),buff+10,8);
	memcpy(&(coordinates.z),buff+18,8);
}

/*Check whether appropriate information was sent from VSP, that finishes communication*/
bool neuron_sensor::transmissionFinished(){
	if(command==0)
		return true;
	return false;
}

Coordinates neuron_sensor::getCoordinates(){
	return coordinates;
}

uint16_t neuron_sensor::getCommand(){
	return command;
}

void neuron_sensor::configure_sensor(){

}

void neuron_sensor::initiate_reading(){
}


} //sensor
} //ecp_mp
} //mrrocpp
