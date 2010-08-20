/**
 * \file datastr.h
 *
 * \date Oct 21, 2009
 * \author ptrojane
 *
 * \brief Methods for MRROC++ data types/string conversion.
 */

#ifndef DATASTR_H_
#define DATASTR_H_

#include <string>

#include "base/lib/com_buf.h"
#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

std::string toString(const double valArr[], int length);
std::string toString(int numberOfPoses);
std::string toString(lib::ECP_POSE_SPECIFICATION ps);
std::string toString(lib::robot_name_t robot);

lib::robot_name_t returnProperRobot(const std::string & robotName);
lib::ECP_POSE_SPECIFICATION returnProperPS(const std::string & poseSpecification);

int setValuesInArray(double arrayToFill[], const std::string & dataString);

}
}

#endif /* DATASTR_H_ */
