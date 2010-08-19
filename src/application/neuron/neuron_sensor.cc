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

#include <ctime>

#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <netinet/in.h>

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

#define NR_OF_TRAJECTORIES		1
#define FIRST_COORDINATES		2
#define TRAJECTORY				3
#define TRAJECTORY_END			4
#define FINISH_COMMUNICATION	5
#define ACTUAL_POSITION			6
#define START_BREAKING			7

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
	counter++;
	/*timespec acttime;
	if( clock_gettime( CLOCK_REALTIME , &acttime) == -1 ){
		printf("sleep generator: next step time measurement error");
	}
	std::cout << acttime.tv_sec << " ";
	std::cout << acttime.tv_nsec <<std::endl;*/

	char buff[25];

	//Read packet from socket*/
	int result = read(socketDescriptor, buff, sizeof(buff));
	if (result < 0) {
		throw std::runtime_error(std::string("read() failed: ") + strerror(errno));
	}

	//check whether whole incoming packet received
	//if (result != sizeof(buff)) {
	//	throw std::runtime_error("read() failed: result != sizeof(MESSAGE_T)");
	//}

	//copy data from packet to variables
	memcpy(&command,buff,1);
	switch(command){
		case NR_OF_TRAJECTORIES:
			memcpy(&(numberOfTrajectories),buff+1,4);
			break;

		case FIRST_COORDINATES:
		case TRAJECTORY:
			memcpy(&(coordinates.x),buff+1,8);
			memcpy(&(coordinates.y),buff+9,8);
			memcpy(&(coordinates.z),buff+17,8);
			printf("coordinates %d, %lf %lf %lf\n",command,coordinates.x,coordinates.y,coordinates.z);
			break;

		case TRAJECTORY_END:
			printf("Neuron Sensor->get_Reading: Trajectory End\n");
			break;

		case FINISH_COMMUNICATION:
			printf("communication finished\n");
			break;

		case START_BREAKING:
			memcpy(&(coordinates.x),buff+1,8);
			memcpy(&(coordinates.y),buff+9,8);
			memcpy(&(coordinates.z),buff+17,8);
			printf("neuron_sensor->get_reading: start breaking %lf %lf %lf",coordinates.x,coordinates.y,coordinates.z);
			break;

		default:
			printf("unknown command %d\n",command);
	}


}

/*Check whether appropriate information was sent from VSP, that finishes communication*/
bool neuron_sensor::transmissionFinished(){
	if(command==TRAJECTORY_END)
		return true;
	return false;
}

Coordinates neuron_sensor::getCoordinates(){
	return coordinates;
}

uint8_t neuron_sensor::getCommand(){
	return command;
}

void neuron_sensor::sendCommand(uint8_t command){
	printf("neuron_sensor->sendCommand: command : %d\n",command);
	int result = write(socketDescriptor, &command, sizeof(uint8_t));

	if (result < 0) {
		throw std::runtime_error(std::string("write() failed: ") + strerror(errno));
	}

	if (result != sizeof(uint8_t)) {
		throw std::runtime_error("write() failed: result != sizeof(uint8_t)");
	}
}

void neuron_sensor::sendCoordinates(double x, double y, double z){
	char buff[25];
	uint8_t temp_command=ACTUAL_POSITION;
	memcpy(buff,&temp_command,1);
	memcpy(buff+1,&x,8);
	memcpy(buff+9,&y,8);
	memcpy(buff+17,&z,8);

	printf("neuron_sensor->sendCoordinates: command : %d x:%lf y:%lf z:%lf\n",temp_command,x,y,z);

	int result=write(socketDescriptor,buff,sizeof(buff));

	if (result < 0) {
		throw std::runtime_error(std::string("write() failed: ") + strerror(errno));
	}

	if (result != sizeof(buff)) {
		throw std::runtime_error("write() failed: result != sizeof(buff)");
	}

}

int neuron_sensor::getNumberOfTrajectories(){
	sendCommand(NR_OF_TRAJECTORIES);
	get_reading();
	return numberOfTrajectories;
}

Coordinates neuron_sensor::getFirstCoordinates(){
	sendCommand(FIRST_COORDINATES);
	get_reading();
	return coordinates;
}

void neuron_sensor::startGettingTrajectory(){
	current_period=1;
	sendCommand(TRAJECTORY);
}

void neuron_sensor::sendCommunicationFinished(){
	sendCommand(FINISH_COMMUNICATION);
}

void neuron_sensor::configure_sensor(){

}

void neuron_sensor::initiate_reading(){
}


} //sensor
} //ecp_mp
} //mrrocpp
