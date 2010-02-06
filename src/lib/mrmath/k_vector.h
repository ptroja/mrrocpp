// ------------------------------------------------------------------------
// Proces:		-
// Plik:			mathtr.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasy K_vector, Homog_matrix, Ft_v_vector,  Jacobian_matrix
//				- definicja klas
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------


#ifndef __K_VECTOR_H
#define __K_VECTOR_H

#include <iostream>

#include "lib/impconst.h"	// frame_tab

namespace mrrocpp {
namespace lib {

// klasa reprezentujaca wektor w kartezjaskim ukaladzie odniesienia
class K_vector
{
private:
	double w[3];


public:
	friend class Homog_matrix;						// klasa Homog_matrix musi miec dostep do prywatnych
															// skladnikow klasy vector

	K_vector ();												// konstruktor domniemany: [0, 0, 0]
	K_vector (double t[3]);								// utworzenie wektora na podstawie tablicy
	K_vector (double x, double y, double z);			// utworzenie wektora na podstawie tablicy
	K_vector (const K_vector &);							// konstruktor kopiujacy


	// Ustawienie elementu wektora.
	void set_value(int i, const double value);
	// Zwrocenie elementu wektora.
	void get_value(int i, double &value) const;
	// Zwrocenie elementu wektora.
	double get_value(int i) const;
	double get_length() const;
	void normalize();

	void to_table(double tablica[3]) const;			// przepisanie zawartosci do tablicy

	K_vector & operator=(const K_vector &);			// operator przypisania
	K_vector & operator=(const double[3]);			// przypisanie tablicy na wektor			- by Slawek Bazant
	K_vector operator+(const K_vector &) const;		// dodawanie wektorow
	K_vector operator-(const K_vector &) const;		// odejmowanie wektorow					- by Slawek Bazant
	K_vector operator*(const K_vector &) const;		// iloczyn wektorowy						- by Slawek Bazant
	K_vector operator*(double) const;					// skalowanie wektora						- by Slawek Bazant
	void operator+=(const K_vector &);				// dodanie wektora podanego jako agument do aktualnego
	void operator*=(const double);					// skalowanie wektora						- by Slawek Bazant

	// in theory, the RHS operator
    double operator[](const int i) const;
    // in theory, the LHS operator
    double& operator[](const int i);

	friend std::ostream& operator<<(std::ostream & s, K_vector & w);	// operator wypisania

};// end class vector



} // namespace lib
} // namespace mrrocpp

#endif
