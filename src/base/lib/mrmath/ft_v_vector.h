#ifndef __FT_V_VECTOR_H
#define __FT_V_VECTOR_H

#include <Eigen/Core>
#include <vector>

namespace mrrocpp {
namespace lib {

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_v_vector : public Eigen::Matrix<double, 6, 1>
{
	typedef Eigen::Matrix<double, 6, 1> BaseClass;

public:
	//! Copy constructor from any Eigen matrix type
	template<typename OtherDerived>
	Ft_v_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: BaseClass(other)
	{}

	//! Reuse assignment operators from base class
	using BaseClass::operator=;

	//! Default constructor
	Ft_v_vector();
	Ft_v_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Ft_v_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	//! Ustawienie elementu wektora.
	void set_values(const double t[6]);										// wypelnienie wektora na podstawie podanej tablicy
	void set_values(double fx, double fy, double fz, double tx, double ty, double tz);

	//! Zwrocenie elementu wektora.
	void to_table(double tablica[6]) const;					// przepisanie wektora do tablicy podanej jako argument

	void to_vector(std::vector<double> & vector);			//fill in the input vector with the coordinates from the actual vector

	//! Wyciagniecie max elementu z wektora
	//! @author Sibi
	double max_element ();	//wyciagniecie maksymalnego elementu wektora

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};// end class Ft_v_vector

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_vector : public Ft_v_vector
{
public:
	template<typename OtherDerived>
	Ft_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: Ft_v_vector(other)
	{}

	Ft_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Ft_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Ft_vector(double fx, double fy, double fz, double tx, double ty, double tz);
};

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Angle_Axis_vector : public Ft_v_vector
{
public:
	template<typename OtherDerived>
	Xyz_Angle_Axis_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: Ft_v_vector(other)
	{}

	Xyz_Angle_Axis_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Xyz_Angle_Axis_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Xyz_Angle_Axis_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	//! Wektor predkosci jako odleglosc dwoch pozycji zadanych w postaci ramek
	//! @author Sibi
	void position_distance(const Homog_matrix& local_current_end_effector_frame, const Homog_matrix& local_desired_end_effector_frame);
};

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Euler_Zyz_vector : public Ft_v_vector
{
public:
	template<typename OtherDerived>
	Xyz_Euler_Zyz_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: Ft_v_vector(other)
	{}

	//! Reuse assignment operators from base class
	using Ft_v_vector::operator=;

	Xyz_Euler_Zyz_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Xyz_Euler_Zyz_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Xyz_Euler_Zyz_vector(double fx, double fy, double fz, double tx, double ty, double tz);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Rpy_vector : public Ft_v_vector
{
public:
	template<typename OtherDerived>
	Xyz_Rpy_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: Ft_v_vector(other)
	{}

	Xyz_Rpy_vector();													// konstruktor domniemany [0, 0, 0, 0, 0, 0]
	Xyz_Rpy_vector(const double t[6]);										// utworzenie wektora na podstawie podanej tablicy
	Xyz_Rpy_vector(double fx, double fy, double fz, double tx, double ty, double tz);
};

} // namespace lib
} // namespace mrrocpp

#endif
