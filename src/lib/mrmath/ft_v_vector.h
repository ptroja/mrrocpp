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

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace mrrocpp {
namespace lib {

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_v_vector : public boost::numeric::ublas::bounded_vector<double, 6>
{
	typedef boost::numeric::ublas::bounded_vector<double, 6> Base_vector;

public:
	// Construction and assignment from a uBLAS vector expression or copy assignment
	template <class R> Ft_v_vector (const boost::numeric::ublas::vector_expression<R>& r) : Base_vector(r)
	{}

	template <class R> void operator=(const boost::numeric::ublas::vector_expression<R>& r)
	{
		Base_vector::operator=(r);
	}

	template <class R> void operator=(const Base_vector& r)
	{
		Base_vector::operator=(r);
	}

	// Default construction
	Ft_v_vector();
	Ft_v_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Ft_v_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	// Ustawienie elementu wektora.
	void set_values(const double t[6]);										// wypelnienie wektora na podstawie podanej tablicy
	void set_values(double fx, double fy, double fz, double tx, double ty, double tz);

	// Zwrocenie elementu wektora.
	void to_table(double tablica[6]) const;					// przepisanie wektora do tablicy podanej jako argument

	//Sibi
	//Wyciagniecie max elementu z wektora
	double max_element ();	//wyciagniecie maksymalnego elementu wektora
};// end class Ft_v_vector



// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_vector : public Ft_v_vector
{
public:
	// Construction and assignment from a uBLAS vector expression or copy assignment
	template <class R> Ft_vector (const boost::numeric::ublas::vector_expression<R>& r) : Ft_v_vector(r)
	{}

	Ft_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Ft_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Ft_vector(double fx, double fy, double fz, double tx, double ty, double tz);
};// end class Ft_vector



// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Angle_Axis_vector : public Ft_v_vector
{
public:
	Xyz_Angle_Axis_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Xyz_Angle_Axis_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Xyz_Angle_Axis_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	//Sibi
	//Wektor predkosci jako odleglosc dwoch pozycji zadanych w postaci ramek
	void position_distance(const Homog_matrix& local_current_end_effector_frame, const Homog_matrix& local_desired_end_effector_frame);
};// end class Xyz_Angle_Axis_vector


// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Euler_Zyz_vector : public Ft_v_vector
{
public:
	Xyz_Euler_Zyz_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Xyz_Euler_Zyz_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Xyz_Euler_Zyz_vector(double fx, double fy, double fz, double tx, double ty, double tz);
};// end class Xyz_Euler_Zyz_vector

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Rpy_vector : public Ft_v_vector
{
public:
	Xyz_Rpy_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Xyz_Rpy_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Xyz_Rpy_vector(double fx, double fy, double fz, double tx, double ty, double tz);
};// end class Xyz_Rpy_vector


} // namespace lib
} // namespace mrrocpp

#endif
