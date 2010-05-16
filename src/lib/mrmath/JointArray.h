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
public:
	JointArray() : Eigen::VectorXd () {}
	JointArray(int size) : Eigen::VectorXd (size) {}
	JointArray(const double *ptr, size_t n) : Eigen::VectorXd (n)
	{
	  for(unsigned int i = 0; i<n; i++)
	  {
		m_storage.data()[i] = ptr[i];
	  }
	}

	typedef Eigen::VectorXd Base;
    template<typename OtherDerived>
    JointArray & operator= (const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }
};

}
}
#endif /* JNTARRAY_H_ */
