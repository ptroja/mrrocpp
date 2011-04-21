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
struct Coordinates
{
	/**
	 * @brief x coordinate.
	 */
	double x;

	/**
	 * @brief y coordinate.
	 */
	double y;

	/**
	 * @brief z coordinate.
	 */
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
class neuron_sensor : public ecp_mp::sensor::sensor_interface
{

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
	 * @brief number of macro steps to perform.
	 */
	uint8_t macroSteps;

	/**
	 * @brief radius of braking circle.
	 */
	double radius;

	/**
	 * Coordinates received from VSP
	 */
	Coordinates coordinates;
	Coordinates lastButOne;

	/**
	 * After how many macrosteps new reading from a sensor should be performed
	 */
	short int basePeriod;

	/**
	 * current counter for basePeriod.
	 */
	short int currentPeriod;

	void sendCommand(uint8_t command);
	void sendData(uint8_t command, double x, double y, double z, double vx, double vy, double vz);

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
	uint8_t getMacroStepsNumber();
	void startGettingTrajectory();
	void sendCommunicationFinished();
	void waitForVSPStart();
	bool startBraking();
	void sendOvershoot(double overshoot);
	void sendRobotState(double x, double y, double z, double vx, double vy, double vz);
	bool newData();
	bool positionRequested();
	void stopReceivingData();
	double getRadius();
};

} //sensor
} //ecp_mp
} //mrrocpp
#endif /* NEURON_SENSOR_H_ */
