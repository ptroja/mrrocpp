#include <sys/types.h>
#include <sys/socket.h>
#include <strings.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

#include "application/wii_teach/sensor/ecp_mp_s_wiimote.h"
#include "base/ecp_mp/ecp_mp_task.h"

#include "lib/exception.h"
#include <boost/throw_exception.hpp>
#include <boost/exception/errinfo_errno.hpp>
#include <boost/exception/errinfo_api_function.hpp>

using namespace std;

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

wiimote::wiimote(lib::sensor::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & config)
	: sr_ecp_msg(_sr_ecp_msg), sensor_name(_sensor_name)
{
	// Set period variables.
	base_period = current_period = 1;

	// Retrieve wiimote node name and port from configuration file.
	int wiimote_port = config.value<int>("wiimote_port", _section_name);
	std::string wiimote_node_name = config.value<std::string>("wiimote_node_name", _section_name);

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
		//buffer used for communication
		char buffer[128];

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
	if (write(sockfd, &to_vsp, sizeof(to_vsp)) == -1) {
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
	to_vsp.i_code = lib::sensor::VSP_INITIATE_READING;

	if (write(sockfd, &to_vsp, sizeof(to_vsp)) == -1) {
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(errno) <<
				lib::exception::error_code(CANNOT_WRITE_TO_DEVICE) <<
				boost::errinfo_api_function("write")
		);
	}
}

void wiimote::send_reading(const wii_command_t & to)
{
	// Send any command to wiimote.
	if (write(sockfd, &to, sizeof(to)) == -1) {
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
	wii_command_t cmd;
	cmd.led_change = false;
	cmd.rumble = false;
	get_reading(cmd);
}

void wiimote::get_reading(const wii_command_t & message)
{
	// Send adequate command to wiimote.
	to_vsp.i_code = lib::sensor::VSP_GET_READING;
	to_vsp.wii_command = message;

	if (write(sockfd, &to_vsp, sizeof(to_vsp)) == -1) {
		BOOST_THROW_EXCEPTION(
			lib::exception::System_error() <<
			boost::errinfo_errno(errno) <<
			lib::exception::error_code(CANNOT_WRITE_TO_DEVICE) <<
			boost::errinfo_api_function("read")
		);
	}

	// Read aggregated data from wiimote.
	if (read(sockfd, &from_vsp, sizeof(from_vsp)) == -1)  {
		BOOST_THROW_EXCEPTION(
			lib::exception::System_error() <<
			boost::errinfo_errno(errno) <<
			lib::exception::error_code(CANNOT_READ_FROM_DEVICE) <<
			boost::errinfo_api_function("read")
		);
	}

	// Check and copy data from buffer to image.
//	if (from_vsp.vsp_report == lib::VSP_REPLY_OK)
	if(from_vsp.vsp_report == lib::sensor::VSP_REPLY_OK)
		image = from_vsp.wiimote;
	else
		sr_ecp_msg.message("Reply from VSP not ok");
}

wiimote::~wiimote()
{
	// Send adequate command to wiimote.
	to_vsp.i_code = lib::VSP_TERMINATE;
	if (write(sockfd, &to_vsp, sizeof(to_vsp)) == -1)  {
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
