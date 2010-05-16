/*
 * JntArray.h
 *
 *  Created on: Jan 6, 2010
 *      Author: konradb3
 */

#ifndef JOINTARRAY_H_
#define JOINTARRAY_H_

#include <Eigen/Core>

namespace mrrocpp {
namespace lib {

class JointArray : public Eigen::VectorXd
{

	typedef Eigen::VectorXd BaseClass;

public:
	JointArray() : Eigen::VectorXd () {}
	JointArray(int size) : Eigen::VectorXd (size) {}
	JointArray(const double *ptr, size_t n) : Eigen::VectorXd (n)
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
#endif /* JNTARRAY_H_ */
