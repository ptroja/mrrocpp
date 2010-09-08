/**
 * \file MotorArray.h
 *
 * \brief Array with numerical for joint-related values
 *
 * \author Konrad Banachowicz <konradb3@gmail.com>
 */

#ifndef JOINTARRAY_H_
#define JOINTARRAY_H_

#include <Eigen/Core>

namespace mrrocpp {
namespace lib {

/**
 * Array with numerical for motor-related values
 *
 * @bug This should be template, parameterized with the array size, for real-time usage. Keep as a header-only class.
 */
class JointArray : public Eigen::VectorXd
{
	//! Typedef for base numerical class
	typedef Eigen::VectorXd BaseClass;

public:
	/**
	 * Constructor
	 */
	JointArray() :
		BaseClass()
	{
	}

	/**
	 * Constructor
	 * \param[in] size size of the array
	 */
	JointArray(int size) :
		BaseClass(size)
	{
	}

	/**
	 * Constructor
	 * \param[in] ptr pointer to the C-array of initialization elements
	 * \param[in] n number of elements in the array
	 */
	JointArray(const double *ptr, size_t n) :
		Eigen::VectorXd(n)
	{
		for (unsigned int i = 0; i < n; i++) {
			this->operator[](i) = ptr[i];
		}
	}

	/**
	 * Assignment operator to reuse from a base class
	 */
	using BaseClass::operator=;
};

}
}
#endif /* JNTARRAY_H_ */
