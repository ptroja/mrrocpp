/**
 * @file neuron_sensor.h
 * @brief Header file for neuron_sensor class.
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup neuron
 * @date 23.06.2010
 */

#ifndef NEURON_SENSOR_H_
#define NEURON_SENSOR_H_


#include "base/ecp_mp/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/**
 * @brief Structure that represents coordinates in 3D space.
 */
struct Coordinates{
	double x;
	double y;
	double z;
};

/**
 * @brief Representation of neuron VSP module in MRROC++.
 * @details In fact class handles communication between MRROC++ and external
 * module which is neuron VSP. It allows sending information about state of
 * the MRROC++ to VSP, sending requests for next position and receving some
 * control commands from VSP which is considered as a server and thus holds
 * main control over entire system.
 */
class neuron_sensor : public ecp_mp::sensor::sensor_interface {

	private:
		/**
		 * @brief Configurator.
		 */
		mrrocpp::lib::configurator& config;

		/**
		 * @brief Socket used to connect with server (VSP).
		 */
		int socketDescriptor;

		/**
		 * @brief command received from VSP.
		 */
		uint8_t command;

		/**
		 * Coordinates received from VSP
		 */
		Coordinates coordinates;
		Coordinates lastButOne;

		void sendCommand(uint8_t command);
		void sendCoordinates(uint8_t command, double x, double y, double z);

	public:
		neuron_sensor(mrrocpp::lib::configurator& _configurator);
		virtual ~neuron_sensor();
		void get_reading();
		void configure_sensor();
		void initiate_reading();
		bool stop();
		uint8_t getCommand();
		Coordinates getFirstCoordinates();
		Coordinates getCoordinates();
		Coordinates getLastButOne();
		void startGettingTrajectory();
		void sendCommunicationFinished();
		void waitForVSPStart();
		bool startBraking();
		void sendFinalPosition(double x, double y, double z);
		void sendCurrentPosition(double x, double y, double z);
};

} //sensor
} //ecp_mp
} //mrrocpp
#endif /* NEURON_SENSOR_H_ */
