/*!
 * \file Xyz_Angle_Axis_Gamma_vector.h
 * \brief Contains the declaration of Xyz_Angle_Axis_Gamma_vector class.
 * \date 12-04-2011
 * \author tkornuta
 */

#ifndef XYZ_ANGLE_AXIS_GAMMA_VECTOR_H_
#define XYZ_ANGLE_AXIS_GAMMA_VECTOR_H_

#include <Eigen/Core>
//#include <vector>

namespace mrrocpp {
namespace lib {

/*!
 * \brief Class containing pose representation denoted by seven parameters: position in the form of XYZ and orientation as vector axis with separated angle.
 * \date 12.04.2011
 * \author tkornuta
 */
class Xyz_Angle_Axis_Gamma_vector : public Eigen::Matrix<double, 7, 1>
{
private:
	//! Helper type for the base type.
	typedef Eigen::Matrix<double, 7, 1> BaseTypeVect7;

public:
	//! Copy constructor from any Eigen matrix type
	template<typename OtherDerived>
	Xyz_Angle_Axis_Gamma_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: BaseTypeVect7(other)
	{}

	//! Default constructor - all elements are zeroed.
	Xyz_Angle_Axis_Gamma_vector();

	//! Constructor that fills the vector on the base of table containing seven doubles.
	Xyz_Angle_Axis_Gamma_vector(const double table_[7]);

	//! Constructor that fills the vector on the base of seven doubles.
	Xyz_Angle_Axis_Gamma_vector(double fx_, double fy_, double fz_, double tx_, double ty_, double tz_, double gamma_);

	//! Destructor. Empty.
	virtual ~Xyz_Angle_Axis_Gamma_vector();

	//! Reuse assignment operators from base class.
	using BaseTypeVect7::operator=;

	// Overload "operator new" so that it generates 16-bytes-aligned pointers.
	// @note not sure if this is really required here.
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}

}

#endif /* XYZ_ANGLE_AXIS_GAMMA_VECTOR_H_ */
