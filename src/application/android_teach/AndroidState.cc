/*
 * AndroidState.cpp
 *
 *  Created on: Dec 17, 2011
 *      Author: hh7
 */

#include "application/android_teach/AndroidState.h"


namespace mrrocpp {
namespace ecp_mp {
namespace sensor {
namespace android_teach {

AndroidState::AndroidState() //: NUMBER_OF_JOINT(7)
{
	// TODO Auto-generated constructor stub
//	this->NUMBER_OF_JOINT = 7;
	connected = false;
}

bool AndroidState::getConnect() const
{
    return connected;
}

//double* AndroidState::getCurrentJointValue()
//{
//    return currentJointValue;
//}
//
//int AndroidState::getMode() const
//{
//    return mode;
//}
//
//double* AndroidState::getNewJointValue()
//{
//    return newJointValue;
//}
//
//int AndroidState::getSpeed() const
//{
//    return speed;
//}

bool AndroidState::getWait() const
{
    return wait;
}

void AndroidState::setConnect(bool connected)
{
    this->connected = connected;
}



//void AndroidState::setMode(int mode)
//{
//    this->mode = mode;
//}
//
//
//void AndroidState::setSpeed(int speed)
//{
//    this->speed = speed;
//}

void AndroidState::setWait(bool wait)
{
    this->wait = wait;
}

int AndroidState::getSelectedAxis() const
{
    return readings.selectedAxis;
}

void AndroidState::setSelectedAxis(int selectedAxis)
{
    this->readings.selectedAxis = selectedAxis;
}



    void AndroidState::updateCurrentPosition(std::vector <double> pos)
    {
        for (int i = 0; i < NUMBER_OF_JOINTS; ++i)
        {
			ecpStatus.jointValue[i] = pos[i];
		}
    }

    void AndroidState::updateCurrentJointPosition(double pos[NUMBER_OF_JOINTS])
    {
        for (int i = 0; i < NUMBER_OF_JOINTS; ++i)
        {
			ecpStatus.jointValue[i] = pos[i];
		}
    }

    void AndroidState::setLimit(int32_t limit)
    {
    	this->ecpStatus.limit = limit;
    }

    int32_t AndroidState::getLimit()
    {
    	return this->ecpStatus.limit;
    }

    void AndroidState::readConfig(lib::configurator &conf, std::vector <double> pos)
    {
        char buffer[100];

        sprintf(buffer,"number_of_speed_intervals");
        config.numberOfSpeedIntervals = conf.value<int>(buffer);

//        sprintf(buffer,"number_of_speed_intervals=%d",config.numberOfSpeedIntervals);
//        perror(buffer);

        for(int i = 0; i < NUMBER_OF_JOINTS; ++i)
        {

        	sprintf(buffer,"joint_max_value_%d",i);
            config.jointMaxValue[i] = conf.value<double>(buffer);

            sprintf(buffer,"joint_min_value_%d",i);
            config.jointMinValue[i] = conf.value<double>(buffer);

            sprintf(buffer,"joint_position_step_%d",i);
            config.positionStep[i] = conf.value<double>(buffer);

            config.jointValue[i] = pos[i];

            sprintf(buffer,"joint_speed_multi_%d",i);
            jointSpeedMulti[i] = conf.value<double>(buffer);

        	sprintf(buffer,"joint[%d] min:%f max:%f step:%f",i,config.jointMinValue[i], config.jointMaxValue[i],config.positionStep[i]);
        	perror(buffer);

//            sprintf(buffer,"blad_%d",i);
//            perror(buffer);

        }


    }


AndroidState::~AndroidState()
{
	// TODO Auto-generated destructor stub
}

} //android_teach
} //sensor
} //ecp_mp
} //mrrocpp

