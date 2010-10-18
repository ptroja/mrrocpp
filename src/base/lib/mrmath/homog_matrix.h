/**
 * \file homog_matrix.h
 *
 * \brief Homogenous transformation matrix
 *
 * \author Piotr Trojanek <piotr.trojanek@gmail.com>
 */

#ifndef __HOMOG_MATRIX_H
#define __HOMOG_MATRIX_H

#include <ostream>
#include <cstring>
#include <cassert>
#include <cmath>

#include "base/lib/impconst.h"	// frame_tab
#define delta_m (M_PI - 3.14154)
#define zero_eps 1.0E-4
#define ALFA_SENSITIVITY 0.00001

namespace mrrocpp {
namespace lib {

// forward declarations
class K_vector;
class Xyz_Angle_Axis_vector;
class Xyz_Euler_Zyz_vector;
class Xyz_Rpy_vector;

//! Turn on or off frames bounds checking.
//! If turned on, assert() can still be turned off with -DNDEBUG.

#define INDEX_CHECK	1

#ifdef INDEX_CHECK
#define HOMOG_MATRIX_CHECKI(a) assert(a)
#else
#define HOMOG_MATRIX_CHECKI(a)
#endif

//! Class for a transformation matrix
class Homog_matrix
{
private:
	//! Matrix placeholder
	frame_tab matrix_m;

	//! Eps for alpha representation
	const static double ALPHA_SENSITIVITY;

public:
	/**
	 * Constructor
	 *
	 * @brief creates an identity matrix
	 */
	Homog_matrix();

	/**
	 * Constructor
	 *
	 * @param[in] frame_tab data for initialization
	 */
	Homog_matrix(const frame_tab &);

	/**
	 * Copy constructor
	 */
	Homog_matrix(const Homog_matrix &);

	/**
	 * Constructor for translation matrix
	 *
	 * @param[in] x,y,z translation parameters
	 */
	Homog_matrix(double x, double y, double z);

	/**
	 * Constructor for a small-rotation around 3 axes
	 *
	 * @param[in] versor_x, versor_y, versor_z versors of X,Y,Z axes
	 * @param[in] angles rotation around 3 axes
	 */
	Homog_matrix(const K_vector & versor_x, const K_vector & versor_y, const K_vector & versor_z, const K_vector & angles);

	/**
	 * Constructor for a small-rotation around 3 axes
	 *
	 * @param[in] angles rotation around 3 axes
	 */
	Homog_matrix(const K_vector & angles);

	/**
	 * Constructor
	 *
	 * @param[in] l_vector
	 */
	Homog_matrix(const Xyz_Euler_Zyz_vector & l_vector);

	/**
	 * Constructor
	 *
	 * @param[in] l_vector
	 */
	Homog_matrix(const Xyz_Rpy_vector & l_vector);

	/**
	 * Constructor
	 *
	 * @param[in] l_vector
	 */
	Homog_matrix(const Xyz_Angle_Axis_vector & l_vector);

	/**
	 * Constructor from Eigen matrix
	 *
	 * @param[in] eigen_matrix matrix for initialization
	 */
	Homog_matrix(const Eigen::Matrix <double, 3, 4> & eigen_matrix);

	/**
	 * Constructor from rotation and translation C-style arrays
	 *
	 * @param[in] r rotation matrix
	 * @param[in] t translation matrix
	 */
	Homog_matrix(const double r[3][3], const double t[3]);

	/**
	 * Constructor from values given in notation from Craig handbook
	 *
	 * @param[in] r??,t? rotation and translation matrix elements
	 */
			Homog_matrix(double r11, double r12, double r13, double t1, double r21, double r22, double r23, double t2, double r31, double r32, double r33, double t3);

	/**
	 * Get the matrix with removed translation
	 *
	 * @return Output matrix
	 */
	Homog_matrix return_with_with_removed_translation() const;

	/**
	 * Get the matrix with removed rotation
	 *
	 * @return Output matrix
	 */
	Homog_matrix return_with_with_removed_rotation() const;

	/**
	 * Get data to frame_tab array
	 *
	 * @param[out] frame output array
	 */
	void get_frame_tab(frame_tab frame) const;

	/**
	 * Set data from frame_tab array
	 *
	 * @param[in] frame output array
	 */
	void set_from_frame_tab(const frame_tab & frame);

	/**
	 * Get the XYZ_EULER_ZYZ representation
	 *
	 * @param[out] l_vector requested representation
	 */
	void get_xyz_euler_zyz(Xyz_Euler_Zyz_vector & l_vector) const;

	/**
	 * Set from the XYZ_EULER_ZYZ representation
	 *
	 * @param[in] l_vector requested representation
	 */
	void set_from_xyz_euler_zyz(const Xyz_Euler_Zyz_vector & l_vector);

	//! Przeksztalcenie do formy XYZ_RPY (rool pitch yaw) i zwrocenie w tablicy.
	void get_xyz_rpy(Xyz_Rpy_vector & l_vector) const;
	// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_RPY.
	void set_from_xyz_rpy(const Xyz_Rpy_vector & l_vector);

	//! Przeksztalcenie do formy XYZ_ANGLE_AXIS i zwrocenie w tablicy.
	void get_xyz_angle_axis(Xyz_Angle_Axis_vector & l_vector) const;

	void set_from_xyz_angle_axis(const Xyz_Angle_Axis_vector & l_vector); // kat wliczony w os

	//! Operacje na kwaternionach
	void set_from_xyz_quaternion(double eta, double eps1, double eps2, double eps3, double x, double y, double z);
	void get_xyz_quaternion(double t[7]) const;

	//! Zwraca obecny wektor translacji.
	void get_translation_vector(double t[3]) const;

	//! wyzerowanie wektora translacji.
	void remove_translation();

	//! wstawienie jedynek na diagonalii rotacji
	void remove_rotation();

	//! Ustawienie wektora translacji. Macierz rotacji pozostaje niezmieniona.
	void set_translation_vector(const double t[3]);

	void set_translation_vector(double x, double y, double z);

	void set_translation_vector(const K_vector & xyz);

	void set_translation_vector(const Homog_matrix & wzor);

	//! Zwrocenie macierzy rotacji.
	void get_rotation_matrix(double r[3][3]) const;
	//! Ustawienie macierzy rotacji. Wektor translacji pozostaje niezmieniony.
	void set_rotation_matrix(const double r[3][3]);

	void set_rotation_matrix(const Homog_matrix &wzor);

	//! Access to elements 0..3,0..2, bounds are checked when NDEBUG is not set
	inline double& operator()(int i, int j)
	{
		HOMOG_MATRIX_CHECKI((0<=i)&&(i<=2)&&(0<=j)&&(j<=3));
		return matrix_m[i][j];
	}

	//! Access to elements 0..3,0..2, bounds are checked when NDEBUG is not set
	inline double operator()(int i, int j) const
	{
		HOMOG_MATRIX_CHECKI((0<=i)&&(i<=2)&&(0<=j)&&(j<=3));
		return matrix_m[i][j];
	}

	//! Operator przypisania.
	Homog_matrix & operator=(const Homog_matrix &);
	//! Mnozenie macierzy.
	Homog_matrix operator*(const Homog_matrix &) const;
	//! Odwracanie macierzy.
	Homog_matrix operator!() const;
	//! Mnozenie macierzy i przypisanie wyniku.
	void operator*=(const Homog_matrix &);

	//! operatory sluzace do przeksztalcania wektorow
	K_vector operator*(const K_vector &) const;
	K_vector operator*(const double tablica[3]) const;

	//! operatory prownania macierzy jednorodnych
	bool operator==(const Homog_matrix &) const;
	bool operator!=(const Homog_matrix &) const;

	//! operator wypisania
	friend std::ostream& operator<<(std::ostream &, const Homog_matrix &);

	//! funkcja sprawdzajaca czy macierz jest macierza jednorodna
	bool is_valid() const;

private:
	//! Kopiowanie macierzy jednorodnej do DEST z SOURCE.
	inline static void copy_frame_tab(frame_tab destination_frame, const frame_tab source_frame)
	{
		std::memcpy(destination_frame, source_frame, sizeof(frame_tab));
	}
};

} // namespace lib
} // namespace mrrocpp

#endif
