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
#include <stdio.h>
#include <iostream>

#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace lib {


// ******************************************************************************************
//                                               definicje skladowych klasy VECTOR
// ******************************************************************************************
K_vector::K_vector()
{
	// konstruktor domniemany
	// tworzy wektor: [0, 0, 0]

	for(int i=0; i<3; i++)
		w[i] = 0;

}// end K_vector::K_vector()

K_vector::K_vector(double t[3])
{
	// utworzenie wektora o wspolrzednych okreslonych przez podana jako argument tablice

	for(int i=0; i<3; i++)
		w[i] = t[i];

}// end K_vector::K_vector(double t[3])

K_vector::K_vector(double x, double y, double z)			// utworzenie wektora na podstawie trzech wsp.
{
	w[0] = x;	w[1] = y; w[2] = z;
}


K_vector::K_vector(const K_vector & wzor)
{
	// konstruktor kopiujacy
	// tworzy wektor o takich samych wspolrzednych jak wektor podany jako argument

	for(int i=0; i<3; i++)
		w[i] = wzor.w[i];

}// end K_vector::K_vector(const K_vector & wzor)


// Ustawienie elementu wektora.
void K_vector::set_value(int i, const double value)
{
	w[i] = value;
}

// Zwrocenie elementu wektora.
void K_vector::get_value(int i, double &value) const
{
	value = w[i];
}

// Zwrocenie dlugosci wektora.
double K_vector::get_length() const
{
	return sqrt (w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
}


// Zwrocenie dligosci wektora.
void K_vector::normalize()
{
	const double length = get_length();
	w[0] /= length;
	w[1] /= length;
	w[2] /= length;
}


// Zwrocenie elementu wektora.
double K_vector::get_value(int i) const
{
	return w[i];
}


void K_vector::to_table(double tablica[3]) const
{
	// przepisanie wespolrzednych wektora do tablicy trzyelementowej

	for(int i=0; i<3;i++)
		tablica[i]=w[i];

}// end K_vector::to_table(double tablica[3]) const

K_vector & K_vector::operator=(const double tablica[3])
{
	// przepisanie tablicy trzyelementowej wspolrzednych do wektora

	for(int i=0; i<3; i++) w[i]=tablica[i];
	return *this;
}// end K_vector::operator=(const double tablica[3]) const

K_vector & K_vector::operator=(const K_vector & p)
{
	// operator przypisania

	if(this == &p) return *this;

	for(int i=0; i<3; i++)
		w[i] = p.w[i];

return *this;
}// end K_vector::operator=(const K_vector & p)

K_vector K_vector::operator+(const K_vector & dod) const
{
	// operator realizujacy dodawanie wektorow

	K_vector zwracany;
	for(int i=0; i<3; i++) zwracany.w[i] = w[i] + dod.w[i];
	return zwracany;
}// end K_vector::operator+(const K_vector & dod) const

K_vector K_vector::operator-(const K_vector &odejmowany) const
{
	// operator realizujacy 	odejmowanie wektorow

	K_vector zwracany;
	for(int i=0; i<3; i++) zwracany.w[i] = w[i] - odejmowany.w[i];
	return zwracany;
}// end K_vector::operator-(const K_vector & dod) const


K_vector K_vector::operator*(const K_vector & iloczyn) const
{
	// operator realizujacy iloczyn wektorowy 2 wektor�w 3x1

	K_vector zwracany;
	zwracany.w[0] = w[1] * iloczyn.w[2] - w[2] * iloczyn.w[1];
	zwracany.w[1] = w[2] * iloczyn.w[0] - w[0] * iloczyn.w[2];
	zwracany.w[2] = w[0] * iloczyn.w[1] - w[1] * iloczyn.w[0];
	return zwracany;
}// end K_vector::operator*(const K_vector & dod) const

K_vector K_vector::operator*(const double skalar) const
{
	// operator realizujacy iloczyn skalarny wektora przez liczbe

    K_vector zwracany;
	for(int i=0;i<3;i++) zwracany.w[i]=w[i]*skalar;
	return zwracany;
}// end K_vector K_vector::operator*(const double skalar) const

void K_vector::operator*=(const double skalar)
{
	// operator realizujacy iloczyn skalarny wektora przez liczbe

	for(int i=0;i<3;i++) w[i] *= skalar;

}// end void K_vector::operator*=(const double skalar)



void K_vector::operator+=(const K_vector & dod)
{
	// operator laczacy w sobie dzialanie dwoch operatorow:
	// - dodawania
	// - przypisania

	for(int i=0; i<3; i++)
		w[i] += dod.w[i];

}// end K_vector::operator+=(const K_vector & dod)

std::ostream& operator<<(std::ostream & s, K_vector & w)
{
	// operator wypisania
	// wypisuje wspolrzedne wektora w przyjaznej dla czlowieka formie

	s << "[ ";
	for(int i=0; i<3; i++)
		s << w.w[i] << " ";
	s << "]\n";
return s;

}// end operator<<(std::ostream & s, K_vector & w)


// in theory, the RHS operator
double K_vector::operator[](const int i) const
{
	//    printf("RHS a[%2d]\n", i );
	return w[i];
}

// in theory, the LHS operator
double& K_vector::operator[](const int i)
{
	// printf("LHS a[%2d]\n", i );
	return w[i];
}




} // namespace lib
} // namespace mrrocpp

