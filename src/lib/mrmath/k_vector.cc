#include "lib/mrmath/k_vector.h"

namespace mrrocpp {
namespace lib {

// ******************************************************************************************
//                                               definicje skladowych klasy VECTOR
// ******************************************************************************************
K_vector::K_vector()
	: BaseClass(BaseClass::Zero())
{
}

K_vector::K_vector(const double t[3])
{
	// utworzenie wektora o wspolrzednych okreslonych przez podana jako argument tablice

	for(int i = 0; i < this->size(); ++i) {
		this->operator[](i) = t[i];
	}
}

K_vector::K_vector(double x, double y, double z)			// utworzenie wektora na podstawie trzech wsp.
{
	this->operator[](0) = x;
	this->operator[](1) = y;
	this->operator[](2) = z;
}

void K_vector::to_table(double tablica[3]) const
{
	// przepisanie wespolrzednych wektora do tablicy trzyelementowej

	for (int i = 0; i < this->size(); ++i)
		tablica[i] = this->operator[](i);
}

} // namespace lib
} // namespace mrrocpp
