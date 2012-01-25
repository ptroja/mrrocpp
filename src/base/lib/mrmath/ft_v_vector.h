#ifndef __FT_V_VECTOR_H
#define __FT_V_VECTOR_H

#include <Eigen/Core>
#include <vector>

#include <boost/serialization/serialization.hpp>

namespace mrrocpp {
namespace lib {

// Forward declaration
class Homog_matrix;

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_v_vector : public Eigen::Matrix<double, 6, 1>
{
	//! Helper type for the base type
	typedef Eigen::Matrix<double, 6, 1> BaseClass;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		for(int r = 0; r < this->rows(); ++r) {
			for(int c = 0; c < this->cols(); ++c) {
				ar & this->operator()(r,c);
			}
		}
	}

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

	//! utworzenie wektora na podstawie podanej tablicy
	Ft_v_vector(const double t[6]);

	//! utworzenie wektora na podstawie wartości
	Ft_v_vector(double fx, double fy, double fz, double tx, double ty, double tz);

	//! przepisanie wektora do tablicy podanej jako argument
	void to_table(double tablica[6]) const;

	//! fill in the input vector with the coordinates from the actual vector
	void to_vector(std::vector<double> & vector);

	//! Wyciagniecie max elementu z wektora
	//! @author Sibi
	double max_element ();

public:
	// overload "operator new" so that it generates 16-bytes-aligned pointers.
	// @note not sure if this is really required here
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Ft_vector : public Ft_v_vector
{
public:
	template<typename OtherDerived>
	Ft_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: Ft_v_vector(other)
	{}

	Ft_vector();
	Ft_vector(const double t[6]);
	Ft_vector(double fx, double fy, double fz, double tx, double ty, double tz);

public:
	// overload "operator new" so that it generates 16-bytes-aligned pointers.
	// @note not sure if this is really required here
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// klasa reprezentujaca wektor sila-moment i wektora predkosci
class Xyz_Angle_Axis_vector : public Ft_v_vector
{
public:
	template<typename OtherDerived>
	Xyz_Angle_Axis_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: Ft_v_vector(other)
	{}

	Xyz_Angle_Axis_vector();
	Xyz_Angle_Axis_vector(const double t[6]);
	Xyz_Angle_Axis_vector(double x, double y, double z, double ax, double ay, double az);

	//! Wektor predkosci jako odleglosc dwoch pozycji zadanych w postaci ramek
	//! @author Sibi
	void position_distance(const Homog_matrix& local_current_end_effector_frame, const Homog_matrix& local_desired_end_effector_frame);

public:
	// overload "operator new" so that it generates 16-bytes-aligned pointers.
	// @note not sure if this is really required here
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

	Xyz_Euler_Zyz_vector();
	Xyz_Euler_Zyz_vector(const double t[6]);
	Xyz_Euler_Zyz_vector(double fx, double fy, double fz, double tx, double ty, double tz);

public:
	// overload "operator new" so that it generates 16-bytes-aligned pointers.
	// @note not sure if this is really required here
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

	Xyz_Rpy_vector();
	Xyz_Rpy_vector(const double t[6]);
	Xyz_Rpy_vector(double fx, double fy, double fz, double tx, double ty, double tz);

public:
	// overload "operator new" so that it generates 16-bytes-aligned pointers.
	// @note not sure if this is really required here
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace lib
} // namespace mrrocpp

#endif
