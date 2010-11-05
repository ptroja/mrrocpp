/*
 * discode_sensor.h
 *
 *  Created on: Oct 30, 2010
 *      Author: mboryn
 */

#ifndef DISCODE_SENSOR_H_
#define DISCODE_SENSOR_H_


#include "base/ecp_mp/ecp_mp_sensor.h"

#include "base/lib/configurator.h"

#include "base/lib/timer.h"

#include <boost/shared_ptr.hpp>

#include "base/lib/xdr/xdr_iarchive.hpp"
#include "base/lib/xdr/xdr_oarchive.hpp"

namespace mrrocpp {

namespace ecp_mp {

namespace sensor {

class discode_sensor : public mrrocpp::ecp_mp::sensor::sensor_interface
{
public:
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
	const std::string& section_name;

	boost::shared_ptr<xdr_iarchive<> > iarchive;
	boost::shared_ptr<xdr_oarchive<> > oarchive;

	mrrocpp::lib::timer timer;
}; // class discode_sensor

} // namespace sensor

} // namespace ecp_mp

} // namespace mrrocpp

#endif /* DISCODE_SENSOR_H_ */
