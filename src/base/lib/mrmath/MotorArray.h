/**
 * \file MotorArray.h
 *
 * \brief Array with numerical for motor-related values
 *
 * \author Konrad Banachowicz <konradb3@gmail.com>
 */

#ifndef MOTORARRAY_H_
#define MOTORARRAY_H_

#include <Eigen/Core>

namespace mrrocpp {
namespace lib {

/**
 * Array with numerical for motor-related values
 *
 * @bug This should be template, parameterized with the array size, for real-time usage. Keep as a header-only class.
 */
class MotorArray : public Eigen::VectorXd
{
	//! Typedef for base numerical class
	typedef Eigen::VectorXd BaseClass;

public:
	/**
	 * Constructor
	 */
	MotorArray() :
		BaseClass()
	{
	}

	/**
	 * Constructor
	 * \param[in] size size of the array
	 */
	MotorArray(int size) :
		BaseClass(size)
	{
	}

	/**
	 * Constructor
	 * \param[in] ptr pointer to the C-array of initialization elements
	 * \param[in] n number of elements in the array
	 */
	MotorArray(const double *ptr, size_t n) :
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
#endif /* MOTORARRAY_H_ */
