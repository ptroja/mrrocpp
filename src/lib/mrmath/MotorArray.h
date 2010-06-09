/*
 * MotorArray.h
 *
 *  Created on: Jan 6, 2010
 *      Author: konradb3
 */

#ifndef MOTORARRAY_H_
#define MOTORARRAY_H_

#include <Eigen/Core>

namespace mrrocpp {
namespace lib {

class MotorArray : public Eigen::VectorXd
{
	
	typedef Eigen::VectorXd BaseClass;

public:
  MotorArray() : Eigen::VectorXd () {}
  MotorArray(int size) : Eigen::VectorXd (size) {}
  MotorArray(const double *ptr, size_t n) : Eigen::VectorXd (n) 
  {
	  for(unsigned int i = 0; i<n; i++)
	  {
		this->operator[](i) = ptr[i];
	  }
  }

  using BaseClass::operator=;

};
}
}
#endif /* MOTORARRAY_H_ */
