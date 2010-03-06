// ------------------------------------------------------------------------
// Proces:		-
// Plik:			mathtr.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasy K_vector, Homog_matrix, Ft_v_vector
//				- deklaracja metod klas
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <math.h>

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

K_vector::K_vector(double t[3])
{
	// utworzenie wektora o wspolrzednych okreslonych przez podana jako argument tablice

	for(int i = 0; i < this->size(); ++i) {
		this->operator[](i) = t[i];
	}
}// end K_vector::K_vector(double t[3])

K_vector::K_vector(double x, double y, double z)			// utworzenie wektora na podstawie trzech wsp.
{
	this->operator[](0) = x;
	this->operator[](1) = y;
	this->operator[](2) = z;
}

// Zwrocenie dlugosci wektora.
double K_vector::get_length() const
{
	const double w0 = this->operator[](0);
	const double w1 = this->operator[](1);
	const double w2 = this->operator[](2);
	return (sqrt(w0*w0 + w1*w1 + w2*w2));
}

// Zwrocenie dligosci wektora.
void K_vector::normalize()
{
	const double length = get_length();
	this->operator[](0) /= length;
	this->operator[](1) /= length;
	this->operator[](2) /= length;
}

void K_vector::to_table(double tablica[3]) const
{
	// przepisanie wespolrzednych wektora do tablicy trzyelementowej

	for (int i = 0; i < this->size(); ++i)
		tablica[i] = this->operator[](i);
}

} // namespace lib
} // namespace mrrocpp

