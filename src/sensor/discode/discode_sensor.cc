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

using namespace std;
using namespace logger;
using namespace boost;

discode_sensor::discode_sensor(mrrocpp::lib::configurator& config, const std::string& section_name) :
	config(config), section_name(section_name)
{
	base_period = current_period = 1;
	timer.print_last_status();
	fflush( stdout);

	oarchive = shared_ptr<xdr_oarchive<> >(new xdr_oarchive<>);
	iarchive = shared_ptr<xdr_iarchive<> >(new xdr_iarchive<>(NULL, 0));
}

discode_sensor::~discode_sensor()
{

}

void discode_sensor::configure_sensor()
{
	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush( stdout);
		throw logic_error("discode_sensor::configure_sensor(): timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

void discode_sensor::initiate_reading()
{
	if (timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED) {
		timer.print_last_status();
		fflush( stdout);
		throw logic_error("discode_sensor::initiate_reading(): timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED");
	}

	float seconds;
	if (timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED) {
		timer.print_last_status();
		fflush( stdout);
		throw logic_error("discode_sensor::initiate_reading(): timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED");
	}
	log_dbg("discode_sensor::initiate_reading() time elapsed: %g s\n", seconds);

	log_dbg("oarchive.getArchiveSize(): %d\n", oarchive->getArchiveSize());


	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush( stdout);
		throw logic_error("discode_sensor::initiate_reading(): timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

void discode_sensor::get_reading()
{
	if (timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED) {
		timer.print_last_status();
		fflush( stdout);
		throw logic_error("discode_sensor::get_reading(): timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED");
	}

	float seconds;
	if (timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED) {
		timer.print_last_status();
		fflush( stdout);
		throw logic_error("discode_sensor::get_reading(): timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED");
	}
	log_dbg("discode_sensor::get_reading() time elapsed: %g s\n", seconds);

	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush( stdout);
		throw logic_error("discode_sensor::get_reading(): timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

void discode_sensor::terminate()
{

}

shared_ptr<xdr_iarchive<> > discode_sensor::get_iarchive()
{
	return iarchive;
}

shared_ptr<xdr_oarchive<> > discode_sensor::get_oarchive()
{
	return oarchive;
}

} // namespace mrrocpp

} // namespace ecp_mp

} // namespace mrrocpp
