/*
 * discode_sensor.cc
 *
 *  Created on: Oct 30, 2010
 *      Author: mboryn
 */

#include <stdexcept>
#include <cstdio>

#include "discode_sensor.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp_mp {

namespace sensor {

namespace discode {

using namespace std;
using namespace logger;
using namespace boost;

discode_sensor::discode_sensor(mrrocpp::lib::configurator& config, const std::string& section_name) :
	config(config), section_name(section_name)
{
	base_period = current_period = 1;
	timer.print_last_status();
	fflush(stdout);

	header_iarchive = shared_ptr <xdr_iarchive <> > (new xdr_iarchive <> );
	iarchive = shared_ptr <xdr_iarchive <> > (new xdr_iarchive <> );
	header_oarchive = shared_ptr <xdr_oarchive <> > (new xdr_oarchive <> );
	oarchive = shared_ptr <xdr_oarchive <> > (new xdr_oarchive <> );
}

discode_sensor::~discode_sensor()
{

}

void discode_sensor::configure_sensor()
{
	// Retrieve FraDIA node name and port from configuration file.
	uint16_t discode_port = configurator.value <uint16_t> ("discode_port", section_name);
	const std::string discode_node_name = configurator.value <std::string> ("discode_node_name", section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		throw std::runtime_error("socket(): " + std::string(strerror(errno)));
	}

	int flag = 1;
	if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int)) == -1) {
		throw std::runtime_error("setsockopt(): " + std::string(strerror(errno)));
	}

	// Get server hostname.
	hostent * server = gethostbyname(discode_node_name.c_str());
	if (server == NULL) {
		throw std::runtime_error("gethostbyname(" + discode_node_name + "): " + std::string(hstrerror(h_errno)));
	}

	// Data with address of connection
	sockaddr_in serv_addr;

	// Reset socketaddr data.
	memset(&serv_addr, 0, sizeof(serv_addr));

	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
	serv_addr.sin_port = htons(discode_port);

	// Try to establish a connection with FraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
		throw std::runtime_error("connect(): " + std::string(strerror(errno)));
	}

	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::configure_sensor(): timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

void discode_sensor::initiate_reading()
{
	if (timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::initiate_reading(): timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED");
	}

	float seconds;
	if (timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::initiate_reading(): timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED");
	}
	log_dbg("discode_sensor::initiate_reading() time elapsed: %g s\n", seconds);

	log_dbg("oarchive.getArchiveSize(): %d\n", oarchive->getArchiveSize());

	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::initiate_reading(): timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

void discode_sensor::get_reading()
{
	if (timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::get_reading(): timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED");
	}

	float seconds;
	if (timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::get_reading(): timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED");
	}
	log_dbg("discode_sensor::get_reading() time elapsed: %g s\n", seconds);

	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::get_reading(): timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

void discode_sensor::terminate()
{
	close(sockfd);
}

shared_ptr <xdr_iarchive <> > discode_sensor::get_iarchive()
{
	return iarchive;
}

shared_ptr <xdr_oarchive <> > discode_sensor::get_oarchive()
{
	return oarchive;
}

} // namespace discode

} // namespace mrrocpp

} // namespace ecp_mp

} // namespace mrrocpp
