/**
 * \file k_vector.h
 *
 * \brief (X,Y,Z) column vector declaration
 *
 * \bug Rename from the polish 'K' prefix
 *
 * \author Piotr Trojanek <piotr.trojanek@gmail.com>
 */

#ifndef __K_VECTOR_H
#define __K_VECTOR_H

#include <Eigen/Core>

namespace mrrocpp {
namespace lib {

/**
 * (X,Y,Z) vector representation
 */
class K_vector : public Eigen::Matrix<double, 3, 1>
{
	//! Base matrix datatype
	typedef Eigen::Matrix<double, 3, 1> BaseClass;

public:
	/**
	 * Copy constructor from any Eigen-inherited matrix type
	 *
	 * @param[in] other initialization matrix
	 */
	template<typename OtherDerived>
	K_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: BaseClass(other)
	{
	}

	//! Reuse assignment operators from base class
	using BaseClass::operator=;

	/**
	 * Constructor for zero vector
	 */
	K_vector ();

	/**
	 * Constructor from C-style array
	 */
	K_vector (const double t[3]);

	/**
	 * Constructor from given values
	 *
	 * @param[in] x,y,z initilization values
	 */
	K_vector (double x, double y, double z);

	/**
	 * Export values to C-style array
	 */
	void to_table(double tablica[3]) const;
};

} // namespace lib
} // namespace mrrocpp

#endif
