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


#ifndef __MATHTR_H
#define __MATHTR_H

#include <iostream>
#include <math.h>
#include <string.h>

#include "lib/impconst.h"	// frame_tab

namespace mrrocpp {
namespace lib {



#define delta_m (M_PI - 3.14154)
#define DEGREES_TO_RADIANS 57.295780

#define zero_eps 1.0E-4

#define ALFA_SENSITIVITY 0.00001


// Sprowadzenie wartosci kata do przedzialu <-pi,pi>
double reduce(double angle);
double reduce(double angle, double min, double max, double offset);


// klasa reprezentujaca wektor w kartezjaskim ukaladzie odniesienia
class K_vector
{
private:
	double w[3];
	Homog_matrix base_frame;										// bazowy trojscian z konstruktora

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



// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_v_vector
{
private:
	double w[6];
public:
	friend class Ft_v_tr;						// klasa Ft_v_tr musi miec dostep do prywatnych
												// skladnikow klasy Ft_v_vector
     friend class Jacobian_matrix;			//Klasa Jacobian_matrix ma miec dostep do skladowych - Sibi

	Ft_v_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Ft_v_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Ft_v_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	Ft_v_vector(const K_vector force, const K_vector torque);

	Ft_v_vector(const Ft_v_vector &);								// konstruktor kopiujacy

	void set_values(const double t[6]);										// wypelnienie wektora na podstawie podanej tablicy
	void set_values(double fx, double fy, double fz, double tx, double ty, double tz);

	// Ustawienie elementu wektora.
	void set_value(int i, const double value);
	// Zwrocenie elementu wektora.
	void get_value(int i, double &value) const;
	// Zwrocenie elementu wektora.
	double get_value(int i) const;

	//Sibi
	 //Wektor predkosci jako odleglosc dwuch pozycji zadanych w postaci ramek
	void position_distance(frame_tab* local_current_end_effector_frame, frame_tab* local_desired_end_effector_frame);


	K_vector get_force_K_vector()  const;
	K_vector get_torque_K_vector()  const;

	// Wyspisanie na ekran wektora
	void wypisz_wartosc_na_konsole() const;

	// in theory, the RHS operator
    double operator[](const int i) const;
      // in theory, the LHS operator
    double& operator[](const int i);

	// Odwracanie macierzy.
	Ft_v_vector operator!() const;
	Ft_v_vector operator-() const;
	Ft_v_vector & operator=(const Ft_v_vector &);			// operator przypisania
	Ft_v_vector operator+(const Ft_v_vector &) const;
	Ft_v_vector operator-(const Ft_v_vector &) const;
	Ft_v_vector operator*(double) const;					// skalowanie wektora
	void operator+=(const Ft_v_vector &);


	void to_table(double tablica[6]) const;					// przepisanie wektora do tablicy podanej jako argument

	friend std::ostream& operator<<(std::ostream & s, Ft_v_vector & w);		// operator wypisania

	//Sibi
	//Wyciagniecie max elementu z wektora
	double max_element ();	//wyciagniecie maksymalnego elementu wektora

};// end class Ft_v_vector


// klasa reprezentujaca macierz transformacji odczytow sily do innego ukladu odniesienia
class Homog_matrix;
class Ft_v_tr
{
public:
	enum VARIANT { FT, V, NOT_SET};

private:
	double matrix_m[6][6];												// zmienna przechowujaca parametry macierzy
	Homog_matrix base_frame;										// bazowy trojscian z konstruktora
	 VARIANT variant;

public:

	Ft_v_tr ();																	// kostruktor domniemany
//	Ft_v_tr(VARIANT variant_l);
	Ft_v_tr (const Homog_matrix &, VARIANT variant_l);
	Ft_v_tr(const Ft_v_tr &);												// konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p);		// ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	Ft_v_tr operator* (const Ft_v_tr & m) const;
	Ft_v_tr operator!() const;

	Ft_v_tr & operator = (const Ft_v_tr &);									// operator przypisania
	Ft_v_vector operator*(const Ft_v_vector &) const;					// mnozenie wektora


	friend std::ostream&  operator<<(std::ostream & strumien, Ft_v_tr &);		// operator wypisania

};// end class Ft_v_tr


//Sibi
// klasa reprezentujaca macierz jakobianu 6 na 6
class Jacobian_matrix
{
private:
	double matrix[6][6];												// Miejsce na jakobian

public:
	Jacobian_matrix ();													// kostruktor domniemany

     void irp6_6dof_equations(const Ft_v_vector & w);			//Wzory na jakobian dla Irp-6 o 6 stopniach swobody
     void irp6_6dof_inverse_equations(const Ft_v_vector & w);				//Wzory na odwrotnosc jakobianu dla Irp-6 o 6 stopniach swobody
	 double irp6_6dof_determinant(const Ft_v_vector & w);					//Wzory na wyznacznik jaokbianu dla Irp-6 o 6 stopniach swobody

     void jacobian_transpose();										//Wyznaczenie transpozycji jakobianu
     void wypisz();														//Wypisanie zawartosci macierzy na konsole
     void to_table(double tablica[6][6]) const;						// przepisanie elementw jakobianu do tablicy[6][6] podanej jako argument

	Ft_v_vector jacobian_inverse_gauss(const Ft_v_vector & dist);				//Rozwiazanie ukladu rownan AX=Y (A, Y - zadane)
																								//za pomoca metody eliminacji Gaussa
	Ft_v_vector operator* (const Ft_v_vector & w) const;		//Przeciazenie operacji mnozenia dla jakobianu i wektora

};// end class Jacobian_matrix

} // namespace lib
} // namespace mrrocpp

#endif
