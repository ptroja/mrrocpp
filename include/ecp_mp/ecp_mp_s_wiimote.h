#ifndef __ECP_WIIMOTE_H
#define __ECP_WIIMOTE_H

#include <netdb.h>

#include "ecp_mp/ecp_mp_sensor.h"

#define BUFFER_SIZE 8*256

class ecp_mp_wiimote_sensor : public sensor
{
private:
	int sockfd;
	sockaddr_in serv_addr;
	hostent* server;
	char buffer[BUFFER_SIZE];
	sr_ecp& sr_ecp_msg;
	SENSOR_ENUM sensor_name;

public:
	ecp_mp_wiimote_sensor (SENSOR_ENUM _sensor_name, const char* _section_name, ecp_mp_task& _ecp_mp_object, int _union_size);
	void configure_sensor (void);
	void initiate_reading (void);
	void send_reading (ECP_VSP_MSG);
	void get_reading (void);
	void terminate();
};
#endif
