/**
 * \file datastr.h
 *
 * \author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * \brief Declarations of methods for MRROC++ data types/string conversion.
 */

#ifndef DATASTR_H_
#define DATASTR_H_

#include <string>

#include "base/lib/com_buf.h"
#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

/**
 * Serialize C-style array to string
 * @param valArr input array
 * @param length number of elements
 * @return string value
 */
std::string toString(const double valArr[], int length);

/**
 * Convert integer value to string
 * @param number input value
 * @return string value
 */
std::string toString(int number);

/**
 * Convert pose specification to string
 * @param ps pose specification type
 * @return string value
 */
std::string toString(lib::ECP_POSE_SPECIFICATION ps);

/**
 * Convert robot name to string
 * @param robot robot name
 * @return string value
 */
std::string toString(const lib::robot_name_t & robot);

/**
 * Convert string description of pose representation to enum value
 * @param robotName input string
 * @return robot name value
 */
lib::robot_name_t returnProperRobot(const std::string & robotName);

/**
 * Convert string description of pose representation to enum value
 * @param poseSpecification input string
 * @return pose representation or ECP_INVALID_END_EFFECTOR in case of error
 * @bug In case error an exception should be thrown
 */
lib::ECP_POSE_SPECIFICATION returnProperPS(const std::string & poseSpecification);

/**
 * Tokenize string to C-style array
 * @param[out] arrayToFill output array
 * @param[in] dataString input string
 * @return number of tokens
 * @bug size of the output array is not checked
 */
unsigned int setValuesInArray(double arrayToFill[], const std::string & dataString);

}
}

#endif /* DATASTR_H_ */
