/*
 * discode_sensor.h
 *
 *  Created on: Oct 30, 2010
 *      Author: mboryn
 */

#ifndef DISCODE_SENSOR_H_
#define DISCODE_SENSOR_H_

#include <string>
#include <cstring>
#include <boost/shared_ptr.hpp>

#include "base/ecp_mp/ecp_mp_sensor.h"
#include "base/lib/configurator.h"
#include "base/lib/timer.h"
#include "base/lib/xdr/xdr_iarchive.hpp"
#include "base/lib/xdr/xdr_oarchive.hpp"

namespace mrrocpp {

namespace ecp_mp {

namespace sensor {

namespace discode {

class discode_sensor : public mrrocpp::ecp_mp::sensor::sensor_interface
{
public:
//	enum discode_sensor_state{
//		NOT_CONFIGURED,
//	};


	discode_sensor(mrrocpp::lib::configurator& config, const std::string& section_name);
	virtual ~discode_sensor();

	virtual void get_reading();

	/**
	 * Virtual method responsible for sensor configuration.
	 */
	virtual void configure_sensor();

	/**
	 * Virtual method responsible for reading initialization.
	 */
	virtual void initiate_reading();

	virtual void terminate();

	boost::shared_ptr<xdr_iarchive<> > get_iarchive();
	boost::shared_ptr<xdr_oarchive<> > get_oarchive();
protected:

private:
	mrrocpp::lib::configurator& config;
	const std::string section_name;

	boost::shared_ptr<xdr_iarchive<> > header_iarchive;
	boost::shared_ptr<xdr_iarchive<> > iarchive;
	boost::shared_ptr<xdr_oarchive<> > header_oarchive;
	boost::shared_ptr<xdr_oarchive<> > oarchive;

	mrrocpp::lib::timer timer;

	/** @brief Socket file descriptor.  */
	int sockfd;

	/** Size of reading_message_header in XDR */
	int reading_message_header_size;

	void timer_init();
	void timer_show(const char *str = "");
}; // class discode_sensor

} // namespace discode

} // namespace sensor

} // namespace ecp_mp

} // namespace mrrocpp

#endif /* DISCODE_SENSOR_H_ */
