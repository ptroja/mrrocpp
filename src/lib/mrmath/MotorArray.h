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
public:
  MotorArray() : Eigen::VectorXd () {}
  MotorArray(int size) : Eigen::VectorXd (size) {}
  MotorArray(const double *ptr, size_t n) : Eigen::VectorXd (n) 
  {
	  for(unsigned int i = 0; i<n; i++)
	  {
		m_storage.data()[i] = ptr[i];
	  }
  }

  typedef Eigen::VectorXd Base;
  template<typename OtherDerived>
  MotorArray & operator= (const Eigen::MatrixBase <OtherDerived>& other)
  {
	this->Base::operator=(other);
	return *this;
  }

};


}
}
#endif /* MOTORARRAY_H_ */
