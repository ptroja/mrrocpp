/*
 * MotorArray.h
 *
 *  Created on: Jan 6, 2010
 *      Author: konradb3
 */

#ifndef MOTORARRAY_H_
#define MOTORARRAY_H_

#include <vector>

namespace mrrocpp {
namespace lib {
class MotorArray : public std::vector<double>
{
public:
  MotorArray() {}
  MotorArray(int size) : std::vector<double> (size) {}
  MotorArray(double *ptr, size_t n) : std::vector<double> (ptr, ptr+n) {}
};

}
}
#endif /* MOTORARRAY_H_ */
