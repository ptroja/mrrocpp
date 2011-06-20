/**
 * \file k_vector.h
 *
 * \brief (X,Y,Z) column vector
 *
 * \bug Rename from the polish 'K' prefix
 *
 * \author Piotr Trojanek <piotr.trojanek@gmail.com>
 */

#include "base/lib/mrmath/k_vector.h"

namespace mrrocpp {
namespace lib {

K_vector::K_vector()
	: BaseClass(BaseClass::Zero())
{
}

K_vector::K_vector(const double t[3])
{
	for(int i = 0; i < this->size(); ++i) {
		this->operator[](i) = t[i];
	}
}

K_vector::K_vector(double x, double y, double z)
{
	this->operator[](0) = x;
	this->operator[](1) = y;
	this->operator[](2) = z;
}

void K_vector::to_table(double tablica[3]) const
{
	for (int i = 0; i < this->size(); ++i)
		tablica[i] = this->operator[](i);
}

} // namespace lib
} // namespace mrrocpp
