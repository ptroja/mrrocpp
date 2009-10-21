/*
 * datastr.h
 *
 *  Created on: Oct 21, 2009
 *      Author: ptroja
 *
 *  methods for MRROC++ data types / string conversion
 */

#ifndef DATASTR_H_
#define DATASTR_H_

#include <string>

#include "lib/com_buf.h"
#include "lib/impconst.h"

namespace mrrocpp {
namespace lib {

std::string toString(double valArr[], int length);
std::string toString(int numberOfPoses);
std::string toString(lib::POSE_SPECIFICATION ps);
std::string toString(lib::ROBOT_ENUM robot);

lib::ROBOT_ENUM returnProperRobot(const std::string & robotName);
lib::POSE_SPECIFICATION returnProperPS(const std::string & poseSpecification);

int setValuesInArray(double arrayToFill[], const char *dataString);

}
}

#endif /* DATASTR_H_ */
