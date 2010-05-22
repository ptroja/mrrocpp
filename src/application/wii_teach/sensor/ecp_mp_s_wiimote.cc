#include <sys/types.h>
#include <sys/socket.h>
#include <strings.h>
#include <unistd.h>
#include <iostream>
#include <string.h>

#include "application/wii_teach/sensor/ecp_mp_s_wiimote.h"
#include "ecp_mp/task/ecp_mp_task.h"

#include "lib/exception.h"
#include <boost/throw_exception.hpp>
#include <boost/exception/errinfo_errno.hpp>
#include <boost/exception/errinfo_api_function.hpp>

using namespace std;

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

wiimote::wiimote(lib::SENSOR_t _sensor_name, const char* _section_name, task::task& _ecp_mp_object, int _union_size) :
	sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg), sensor_name(_sensor_name)
{
	// Set size of passed message/union.
	union_size = _union_size + sizeof(image.sensor_union.begin);

	// Set period variables.
	base_period = current_period = 1;

	// Retrieve wiimote node name and port from configuration file.
	int wiimote_port = _ecp_mp_object.config.value <int> ("wiimote_port", _section_name);
	std::string wiimote_node_name = _ecp_mp_object.config.value <std::string> ("wiimote_node_name", _section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(errno) <<
				boost::errinfo_api_function("socket")
		);
	}

	// Get server hostname.
	server = gethostbyname(wiimote_node_name.c_str());
	if (server == NULL) {
		sprintf(buffer, "ERROR, no host %s", wiimote_node_name.c_str());
		sr_ecp_msg.message(buffer);
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				lib::exception::h_errno_code(h_errno) <<
				lib::exception::error_code(CANNOT_LOCATE_DEVICE) <<
				boost::errinfo_api_function("gethostbyname")
		);
	}

	// Reset socketaddr data.
	bzero((char *) &serv_addr, sizeof(serv_addr));
	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	bcopy((char *) server->h_addr, (char *) &serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(wiimote_port);

	// Try to establish a connection with wiimote.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(errno) <<
				lib::exception::error_code(CANNOT_LOCATE_DEVICE) <<
				boost::errinfo_api_function("connect")
		);
	}

	sr_ecp_msg.message("Connected to wiimote");
}

void wiimote::configure_sensor()
{
	// Send adequate command to wiimote.
	to_vsp.i_code = lib::VSP_CONFIGURE_SENSOR;
	if (write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1) {
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(errno) <<
				lib::exception::error_code(CANNOT_WRITE_TO_DEVICE) <<
				boost::errinfo_api_function("write")
		);
	}
}

void wiimote::initiate_reading()
{
	// Send adequate command to wiimote.
	to_vsp.i_code = lib::VSP_INITIATE_READING;

	if (write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1) {
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(errno) <<
				lib::exception::error_code(CANNOT_WRITE_TO_DEVICE) <<
				boost::errinfo_api_function("write")
		);
	}
}

void wiimote::send_reading(lib::ECP_VSP_MSG to)
{
	// Send any command to wiimote.

	if (write(sockfd, &to, sizeof(lib::ECP_VSP_MSG)) == -1) {
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(errno) <<
				lib::exception::error_code(CANNOT_WRITE_TO_DEVICE) <<
				boost::errinfo_api_function("write")
		);
	}
}

void wiimote::get_reading()
{
	// Send adequate command to wiimote.
	to_vsp.i_code = lib::VSP_GET_READING;
	to_vsp.wii_command.led_change = false;
	to_vsp.wii_command.rumble = false;
	get_reading(to_vsp);
}

void wiimote::get_reading(lib::ECP_VSP_MSG message)
{
	// Send adequate command to wiimote.
	to_vsp.i_code = lib::VSP_GET_READING;
	to_vsp.wii_command = message.wii_command;

	if (write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1) {
		BOOST_THROW_EXCEPTION(
			lib::exception::System_error() <<
			boost::errinfo_errno(errno) <<
			lib::exception::error_code(CANNOT_WRITE_TO_DEVICE) <<
			boost::errinfo_api_function("read")
		);
	}

	// Read aggregated data from wiimote.
	if (read(sockfd, &from_vsp, sizeof(lib::VSP_ECP_MSG)) == -1)  {
		BOOST_THROW_EXCEPTION(
			lib::exception::System_error() <<
			boost::errinfo_errno(errno) <<
			lib::exception::error_code(CANNOT_READ_FROM_DEVICE) <<
			boost::errinfo_api_function("read")
		);
	}

	// Check and copy data from buffer to image.
	if (from_vsp.vsp_report == lib::VSP_REPLY_OK)
		memcpy(&(image.sensor_union.begin), &(from_vsp.comm_image.sensor_union.begin), union_size);
	else
		sr_ecp_msg.message("Reply from VSP not ok");
}

wiimote::~wiimote()
{
	// Send adequate command to wiimote.
	to_vsp.i_code = lib::VSP_TERMINATE;
	if (write(sockfd, &to_vsp, sizeof(lib::ECP_VSP_MSG)) == -1)  {
		BOOST_THROW_EXCEPTION(
			lib::exception::System_error() <<
			boost::errinfo_errno(errno) <<
			lib::exception::error_code(CANNOT_WRITE_TO_DEVICE) <<
			boost::errinfo_api_function("write")
		);
	}

	close(sockfd);
	sr_ecp_msg.message("Terminate\n");
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
