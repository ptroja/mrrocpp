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


#ifndef __FT_V_VECTOR_H
#define __FT_V_VECTOR_H

#include <iostream>
#include <math.h>
#include <string.h>

#include "lib/impconst.h"	// frame_tab

namespace mrrocpp {
namespace lib {



// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_v_vector
{
public:
	double w[6];
	friend class Ft_v_tr;						// klasa Ft_v_tr musi miec dostep do prywatnych
	friend class Ft_tr;
	friend class V_tr;
    friend class Jacobian_matrix;			//Klasa Jacobian_matrix ma miec dostep do skladowych - Sibi


	Ft_v_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Ft_v_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Ft_v_vector(double fx, double fy, double fz, double tx, double ty, double tz);


	Ft_v_vector(const Ft_v_vector &);								// konstruktor kopiujacy

	void set_values(const double t[6]);										// wypelnienie wektora na podstawie podanej tablicy
	void set_values(double fx, double fy, double fz, double tx, double ty, double tz);

	// Ustawienie elementu wektora.
	void set_value(int i, const double value);
	// Zwrocenie elementu wektora.
	void get_value(int i, double &value) const;
	// Zwrocenie elementu wektora.
	double get_value(int i) const;

	// Wyspisanie na ekran wektora
	void wypisz_wartosc_na_konsole() const;

	// in theory, the RHS operator
    double operator[](const int i) const;
      // in theory, the LHS operator
    double& operator[](const int i);

	void operator+=(const Ft_v_vector &);

	void to_table(double tablica[6]) const;					// przepisanie wektora do tablicy podanej jako argument

	friend std::ostream& operator<<(std::ostream & s, Ft_v_vector & w);		// operator wypisania

	//Sibi
	//Wyciagniecie max elementu z wektora
	double max_element ();	//wyciagniecie maksymalnego elementu wektora


	// Odwracanie macierzy.
	Ft_v_vector operator!() const;
	Ft_v_vector operator-() const;
	Ft_v_vector & operator=(const Ft_v_vector &);			// operator przypisania
	Ft_v_vector operator+(const Ft_v_vector &) const;
	Ft_v_vector operator-(const Ft_v_vector &) const;
	Ft_v_vector operator*(double) const;					// skalowanie wektora

};// end class Ft_v_vector



// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_vector : public Ft_v_vector
{

public:

												// skladnikow klasy Ft_v_vector

	Ft_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Ft_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Ft_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	Ft_vector(const Ft_vector &);								// konstruktor kopiujacy

	// Odwracanie macierzy.
	Ft_vector operator!() const;
	Ft_vector operator-() const;
	Ft_vector & operator=(const Ft_vector &);			// operator przypisania
	Ft_vector operator+(const Ft_vector &) const;
	Ft_vector operator-(const Ft_vector &) const;
	Ft_vector operator*(double) const;					// skalowanie wektora
};// end class Ft_vector



// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Angle_Axis_vector : public Ft_v_vector
{

public:

	Xyz_Angle_Axis_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Xyz_Angle_Axis_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Xyz_Angle_Axis_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	Xyz_Angle_Axis_vector(const Xyz_Angle_Axis_vector &);								// konstruktor kopiujacy

	// Odwracanie macierzy.
	Xyz_Angle_Axis_vector operator!() const;
	Xyz_Angle_Axis_vector operator-() const;
	Xyz_Angle_Axis_vector & operator=(const Xyz_Angle_Axis_vector &);			// operator przypisania
	Xyz_Angle_Axis_vector operator+(const Xyz_Angle_Axis_vector &) const;
	Xyz_Angle_Axis_vector operator-(const Xyz_Angle_Axis_vector &) const;
	Xyz_Angle_Axis_vector operator*(double) const;					// skalowanie wektora

	//Sibi
	 //Wektor predkosci jako odleglosc dwoch pozycji zadanych w postaci ramek
	void position_distance(Homog_matrix& local_current_end_effector_frame, Homog_matrix& local_desired_end_effector_frame);


};// end class Xyz_Angle_Axis_vector


// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Euler_Zyz_vector : public Ft_v_vector
{

public:

	Xyz_Euler_Zyz_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Xyz_Euler_Zyz_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Xyz_Euler_Zyz_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	Xyz_Euler_Zyz_vector(const Xyz_Euler_Zyz_vector &);								// konstruktor kopiujacy

	// Odwracanie macierzy.
	Xyz_Euler_Zyz_vector operator!() const;
	Xyz_Euler_Zyz_vector operator-() const;
	Xyz_Euler_Zyz_vector & operator=(const Xyz_Euler_Zyz_vector &);			// operator przypisania
	Xyz_Euler_Zyz_vector operator+(const Xyz_Euler_Zyz_vector &) const;
	Xyz_Euler_Zyz_vector operator-(const Xyz_Euler_Zyz_vector &) const;
	Xyz_Euler_Zyz_vector operator*(double) const;					// skalowanie wektora

};// end class Xyz_Euler_Zyz_vector

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Rpy_vector : public Ft_v_vector
{

public:

	Xyz_Rpy_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Xyz_Rpy_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Xyz_Rpy_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	Xyz_Rpy_vector(const Xyz_Rpy_vector &);								// konstruktor kopiujacy

	// Odwracanie macierzy.
	Xyz_Rpy_vector operator!() const;
	Xyz_Rpy_vector operator-() const;
	Xyz_Rpy_vector & operator=(const Xyz_Rpy_vector &);			// operator przypisania
	Xyz_Rpy_vector operator+(const Xyz_Rpy_vector &) const;
	Xyz_Rpy_vector operator-(const Xyz_Rpy_vector &) const;
	Xyz_Rpy_vector operator*(double) const;					// skalowanie wektora

};// end class Xyz_Rpy_vector


} // namespace lib
} // namespace mrrocpp

#endif
