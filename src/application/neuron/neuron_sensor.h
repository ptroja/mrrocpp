/*
 * neuron_sensor.h
 *
 *  Created on: Jun 23, 2010
 *      Author: tbem
 */

#ifndef NEURON_SENSOR_H_
#define NEURON_SENSOR_H_


#include "base/ecp_mp/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

struct Coordinates{
	double x;
	double y;
	double z;
};
/*
 * Class responsible for representing neuron vsp in mrrocpp that in fact handles communication between
 * mrrocpp and external vsp
 */
class neuron_sensor : public ecp_mp::sensor::sensor_interface {

	private:
		/** Configurator. */
		mrrocpp::lib::configurator& config;
		int socketDescriptor;
		uint8_t command;
		int numberOfTrajectories;
		Coordinates coordinates;
		void sendCommand(uint8_t command);

	public:
		neuron_sensor(mrrocpp::lib::configurator& _configurator);
		virtual ~neuron_sensor();
		void get_reading();
		void configure_sensor();
		void initiate_reading();
		bool transmissionFinished();
		uint8_t getCommand();
		int getNumberOfTrajectories();
		Coordinates getFirstCoordinates();
		Coordinates getCoordinates();
		void startGettingTrajectory();
		void sendCommunicationFinished();
		void waitForVSPStart();
		bool startBraking();
		void sendCoordinates(double x, double y, double z);
};

} //sensor
} //ecp_mp
} //mrrocpp
#endif /* NEURON_SENSOR_H_ */
