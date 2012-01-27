/*
 * RobotState.h
 *
 *  Created on: Dec 17, 2011
 *      Author: hh7
 */

#ifndef ANDROIDSTATE_H_
#define ANDROIDSTATE_H_


#include "application/android_teach/Enums.h"
#include "base/ecp_mp/ecp_mp_task.h"
#include <vector>

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {
namespace android_teach {


class AndroidState
{
public:
//	int mode;
//	int value;
//	int selectedAxis;
//	double currentJointValue[NUMBER_OF_JOINTS];
//	double newJointValue[NUMBER_OF_JOINTS];
//	double jointMaxValue[NUMBER_OF_JOINTS];
//	double jointMinValue[NUMBER_OF_JOINTS];
//	double positionStep[NUMBER_OF_JOINTS];
//	int32_t numberOfSpeedIntervals;
	EcpStatus ecpStatus;
	Configuration config;
	Readings readings;


	bool wait;
	bool connected;
	bool disconnected;
	double jointSpeedMulti[NUMBER_OF_JOINTS];



public:
//	const int NUMBER_OF_JOINT;

//
    int getSelectedAxis() const;
    void setSelectedAxis(int selectedAxis);
    bool getConnect() const;
//    double* getCurrentJointValue();
//    int getMode() const;
//    double* getNewJointValue();
//    int getSpeed() const;
    bool getWait() const;
    void setConnect(bool connect);
//    void setMode(int mode);
//    void setSpeed(int speed);
    void setWait(bool wait);

    void setLimit(int32_t limit);
    int32_t getLimit();

    void updateCurrentPosition(std::vector <double> pos);
    void updateCurrentJointPosition(double pos[NUMBER_OF_JOINTS]);


    void readConfig(lib::configurator &conf, std::vector <double> pos);

    AndroidState();
	virtual ~AndroidState();


//	int getSpeed();
//	void setSpeed(int value);
//	int getMode();
//	void setMode(int value);



};

} //android_teach
} //sensor
} //ecp_mp
} //mrrocpp

#endif /* ROBOTSTATE_H_ */
