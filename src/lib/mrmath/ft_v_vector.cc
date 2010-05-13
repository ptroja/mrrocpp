#include <math.h>

#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace lib {

// ******************************************************************************************
//                                           definicje skladowych klasy Ft_v_vector
// ******************************************************************************************

Ft_v_vector::Ft_v_vector()
	: BaseClass(BaseClass::Zero())
{
}

Ft_v_vector::Ft_v_vector(const double t[6])
{
	// utworznie wektora o parametrach podanych w tablicy, bedacej argumentem
	set_values(t);
}

Ft_v_vector::Ft_v_vector(double fx, double fy, double fz, double tx, double ty, double tz)
{
	// utworznie wektora o parametrach podanych w tablicy, bedacej argumentem
	set_values(fx, fy, fz, tx, ty, tz);
}

// Ustawienie elementu wektora.
void Ft_v_vector::set_values(const double t[6])
{
	for(int i = 0; i < this->size(); ++i) {
		this->operator[](i) = t[i];
	}
}

// Ustawienie elementu wektora.
void Ft_v_vector::set_values(double fx, double fy, double fz, double tx, double ty, double tz)
{
	this->operator[](0) = fx;
	this->operator[](1) = fy;
	this->operator[](2) = fz;
	this->operator[](3) = tx;
	this->operator[](4) = ty;
	this->operator[](5) = tz;
}

void Ft_v_vector::to_table(double tablica[6]) const
{
	// przepisanie parametrow wektora do szescioelementowej tablicy podanej jako argument

	for (int i = 0; i < this->size(); ++i)
		tablica[i] = this->operator[](i);
}

//Sibi
// Wyciadgniecie maksymalnego elementu z zadanego wektora
double Ft_v_vector::max_element()
{
	// TODO: Eigen'ize this method, probably:
	// return BaseClass::cwise().abs().maxCoeff();
	double MAX = 0;

	for (int i = 0; i < this->size(); ++i) {
		if (fabs(operator[](i)) > MAX) {
			MAX = fabs(operator[](i));
		}
	}

	return MAX;
}

/////////////////////////////////////////
//
//  Ft_vector
//
//////////////////////////////////////////


Ft_vector::Ft_vector() :
	Ft_v_vector()
{
}

Ft_vector::Ft_vector(const double t[6]) :
	Ft_v_vector(t)
{
}

Ft_vector::Ft_vector(double fx, double fy, double fz, double tx, double ty, double tz) :
	Ft_v_vector(fx, fy, fz, tx, ty, tz)
{
}

/////////////////////////////////////////
//
//  Xyz_Angle_Axis_vector
//
//////////////////////////////////////////


Xyz_Angle_Axis_vector::Xyz_Angle_Axis_vector() :
	Ft_v_vector()
{
}

Xyz_Angle_Axis_vector::Xyz_Angle_Axis_vector(const double t[6]) :
	Ft_v_vector(t)
{
}

Xyz_Angle_Axis_vector::Xyz_Angle_Axis_vector(double fx, double fy, double fz, double tx, double ty, double tz) :
	Ft_v_vector(fx, fy, fz, tx, ty, tz)
{
}

void Xyz_Angle_Axis_vector::position_distance(const Homog_matrix& local_current_end_effector_frame, const Homog_matrix& local_desired_end_effector_frame)
{
	double n_t[3], n_d[3], o_t[3], o_d[3], a_t[3], a_d[3];

	//Wyliczenie wektora przesuniecia : w - predkosc katowa
	// n, o, a - wektory normalny, orientacji i zbli�enia

	n_t[0] = local_current_end_effector_frame(0,0);
	n_t[1] = local_current_end_effector_frame(1,0);
	n_t[2] = local_current_end_effector_frame(2,0);

	n_d[0] = local_desired_end_effector_frame(0,0);
	n_d[1] = local_desired_end_effector_frame(1,0);
	n_d[2] = local_desired_end_effector_frame(2,0);

	o_t[0] = local_current_end_effector_frame(0,1);
	o_t[1] = local_current_end_effector_frame(1,1);
	o_t[2] = local_current_end_effector_frame(2,1);

	o_d[0] = local_desired_end_effector_frame(0,1);
	o_d[1] = local_desired_end_effector_frame(1,1);
	o_d[2] = local_desired_end_effector_frame(2,1);

	a_t[0] = local_current_end_effector_frame(0,2);
	a_t[1] = local_current_end_effector_frame(1,2);
	a_t[2] = local_current_end_effector_frame(2,2);

	a_d[0] = local_desired_end_effector_frame(0,2);
	a_d[1] = local_desired_end_effector_frame(1,2);
	a_d[2] = local_desired_end_effector_frame(2,2);

	//Wyliczenie wektora przesuniecia : v - predkosc obrotowa

	this->operator[](0) = (0.5) * ((n_t[1] * n_d[2] - n_t[2] * n_d[1]) + (o_t[1] * o_d[2] - o_t[2] * o_d[1]) + (a_t[1] * a_d[2]
			- a_t[2] * a_d[1]));
	this->operator[](1) = (0.5) * ((n_t[2] * n_d[0] - n_t[0] * n_d[2]) + (o_t[2] * o_d[0] - o_t[0] * o_d[2]) + (a_t[2] * a_d[0]
			- a_t[0] * a_d[2]));
	this->operator[](2) = (0.5) * ((n_t[0] * n_d[1] - n_t[1] * n_d[0]) + (o_t[0] * o_d[1] - o_t[1] * o_d[0]) + (a_t[0] * a_d[1]
			- a_t[1] * a_d[0]));

	//Wyliczenie wektora przesuniecia : v - predkosc liniowa

	this->operator[](3) = local_desired_end_effector_frame(0,3) - local_current_end_effector_frame(0,3);
	this->operator[](4) = local_desired_end_effector_frame(1,3) - local_current_end_effector_frame(1,3);
	this->operator[](5) = local_desired_end_effector_frame(2,3) - local_current_end_effector_frame(2,3);
}

/////////////////////////////////////////
//
//  Xyz_Euler_Zyz_vector
//
//////////////////////////////////////////


Xyz_Euler_Zyz_vector::Xyz_Euler_Zyz_vector() :
	Ft_v_vector()
{
}

Xyz_Euler_Zyz_vector::Xyz_Euler_Zyz_vector(const double t[6]) :
	Ft_v_vector(t)
{
}

Xyz_Euler_Zyz_vector::Xyz_Euler_Zyz_vector(double fx, double fy, double fz, double tx, double ty, double tz) :
	Ft_v_vector(fx, fy, fz, tx, ty, tz)
{
}

/////////////////////////////////////////
//
//  Xyz_Rpy_vector
//
//////////////////////////////////////////


Xyz_Rpy_vector::Xyz_Rpy_vector() :
	Ft_v_vector()
{
}

Xyz_Rpy_vector::Xyz_Rpy_vector(const double t[6]) :
	Ft_v_vector(t)
{
}

Xyz_Rpy_vector::Xyz_Rpy_vector(double fx, double fy, double fz, double tx, double ty, double tz) :
	Ft_v_vector(fx, fy, fz, tx, ty, tz)
{
}

} // namespace lib
} // namespace mrrocpp

