/*!
 * \file Xyz_Angle_Axis_Gamma_vector.cc
 * \brief Contains the definition of Xyz_Angle_Axis_Gamma_vector class methods.
 * \date 12-04-2011
 * \author tkornuta
 */

#include "Xyz_Angle_Axis_Gamma_vector.h"

namespace mrrocpp {
namespace lib {


Xyz_Angle_Axis_Gamma_vector::Xyz_Angle_Axis_Gamma_vector()
	: BaseTypeVect7(BaseTypeVect7::Zero())
{
}


Xyz_Angle_Axis_Gamma_vector::Xyz_Angle_Axis_Gamma_vector(const double table_[7])
	: BaseTypeVect7(table_)
{
}

Xyz_Angle_Axis_Gamma_vector::Xyz_Angle_Axis_Gamma_vector(double fx_, double fy_, double fz_, double tx_, double ty_, double tz_, double gamma_)
{
	(*this) << fx_, fy_, fz_, tx_, ty_, tz_, gamma_;
}

Xyz_Angle_Axis_Gamma_vector::~Xyz_Angle_Axis_Gamma_vector()
{
}


}//: namespace lib
}//: namespace mrrocpp
