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
#include <cassert>

#include <string>

#include <Eigen/Core>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>

namespace mrrocpp {
namespace lib {

// forward declarations
class K_vector;
class Xyz_Angle_Axis_vector;
class Xyz_Angle_Axis_Gamma_vector;
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
	double matrix_m[3][4];

	//! Eps for alpha representation
	const static double ALPHA_SENSITIVITY;

public:
	/*!
	 * Constructor
	 *
	 * @brief creates an identity matrix
	 */
	Homog_matrix();

	/*!
	 * Constructor for translation matrix
	 *
	 * @param[in] x,y,z translation parameters
	 */
	Homog_matrix(double x, double y, double z);

	/*!
	 * Constructor for a small-rotation around 3 axes
	 *
	 * @param[in] versor_x, versor_y, versor_z versors of X,Y,Z axes
	 * @param[in] angles rotation around 3 axes
	 */
	Homog_matrix(const K_vector & versor_x, const K_vector & versor_y, const K_vector & versor_z, const K_vector & angles);

	/*!
	 * Constructor for a small-rotation around 3 axes
	 *
	 * @param[in] angles rotation around 3 axes
	 */
	Homog_matrix(const K_vector & angles);

	/*!
	 * Constructor
	 *
	 * @param[in] l_vector
	 */
	Homog_matrix(const Xyz_Euler_Zyz_vector & l_vector);

	/*!
	 * Constructor
	 *
	 * @param[in] l_vector
	 */
	Homog_matrix(const Xyz_Rpy_vector & l_vector);

	/*!
	 * Constructor
	 *
	 * @param[in] l_vector
	 */
	Homog_matrix(const Xyz_Angle_Axis_vector & l_vector);

	/*!
	 * Constructor from Eigen matrix (reduced 3x4 matrix).
	 *
	 * @param[in] Eigen-based matrix for initialization.
	 */
	Homog_matrix(const Eigen::Matrix <double, 3, 4> & eigen_matrix);

	/*!
	 * Constructor from Eigen matrix (full 4x4 matrix).
	 *
	 * @param[in] Eigen-based matrix for initialization.
	 */
	Homog_matrix(const Eigen::Matrix <double, 4, 4> & eigen_matrix);

	/*!
	 * Constructor from rotation and translation C-style arrays
	 *
	 * @param[in] r rotation matrix
	 * @param[in] t translation matrix
	 */
	Homog_matrix(const double r[3][3], const double t[3]);

	/*!
	 * Constructor from values given in notation from Craig handbook
	 *
	 * @param[in] r??,t? rotation and translation matrix elements
	 */
	Homog_matrix(double r11, double r12, double r13, double t1, double r21, double r22, double r23, double t2, double r31, double r32, double r33, double t3);

	/*!
	 * Constructor from values given as Matlab-style matrix string
	 *
	 * @param[in] s string in the form "[ r11 r12 r13 t1; r21 r22 r23 t2; r31 r32 r33 t3; 0 0 0 1 ]"
	 */
	Homog_matrix(const std::string & s);

	/*!
	 * Sets identity matrix.
	 */
	void setIdentity();

	/*!
	 * Sets values basing on passed string.
	 */
	void set(const std::string & str);

	/*!
	 * Get the matrix with removed translation
	 *
	 * @return Output matrix
	 */
	Homog_matrix return_with_with_removed_translation() const;

	/*!
	 * Get the matrix with removed rotation
	 *
	 * @return Output matrix
	 */
	Homog_matrix return_with_with_removed_rotation() const;

	/**
	 * Get the XYZ_EULER_ZYZ representation
	 *
	 * @param[out] l_vector requested representation.
	 */
	void get_xyz_euler_zyz(Xyz_Euler_Zyz_vector & l_vector) const;

	/**
	 * Get the XYZ_EULER_ZYZ representation without limits for beta (it can vary from <-PI, PI)).
	 *
	 * @param[out] l_vector requested representation.
	 * @param [in] alpha_old - previous value used for solution selection.
	 * @param [in] beta_old - previous value used for solution selection.
	 * @param [in] gamma_old - previous value used for solution selection.
	 */
	void get_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector & l_vector, const double alfa, const double beta, const double gamma) const;

	/*!
	 * Set from the XYZ_EULER_ZYZ representation. Takes into consideration limits for beta <0, PI).
	 *
	 * @param[in] l_vector requested representation.
	 */
	void set_from_xyz_euler_zyz(const Xyz_Euler_Zyz_vector & l_vector);

	/*!
	 * Set from the XYZ_EULER_ZYZ representation without limits for beta (it can vary from <-PI, PI)).
	 *
	 * @param[in] l_vector requested representation.
	 */
	void set_from_xyz_euler_zyz_without_limits(const Xyz_Euler_Zyz_vector & l_vector);

	/*!
	 * Computes the angle and axis values for given homogeneous matrix.
	 * @param[out] xyz_aa Vector containing computed pose (x,y,z) and orientation (vx,vy,vz,gamma).
	 */
	void get_xyz_angle_axis_gamma(Xyz_Angle_Axis_Gamma_vector & xyz_aa_gamma) const;

	//! Returns XYZ_ANGLE_AXIS representation (x,y,z, vx*gamma,vy*gamma,vz*gamma).
	void get_xyz_angle_axis(Xyz_Angle_Axis_vector & xyz_aa) const;

	//! Sets the homogeneous matrix values on the base of given XYZ_ANGLE_AXIS vector.
	void set_from_xyz_angle_axis(const Xyz_Angle_Axis_vector & xyz_aa);

	//! Sets the homogeneous matrix values on the base of given XYZ_ANGLE_AXIS (x,y,z, vx,vy,vz,gamma) vector.
	void set_from_xyz_angle_axis_gamma(const Xyz_Angle_Axis_Gamma_vector & xyz_aa_gamma);


	//! Przeksztalcenie do formy XYZ_RPY (rool pitch yaw) i zwrocenie w tablicy.
	void get_xyz_rpy(Xyz_Rpy_vector & l_vector) const;

	//! Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_RPY.
	void set_from_xyz_rpy(const Xyz_Rpy_vector & l_vector);

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

	void get_rotation_matrix(Eigen::Matrix3d& rot) const;
	void set_rotation_matrix(const Eigen::Matrix3d& rot);

	/**
	 * Linear-spherical interpolation of Homog matrix.
	 * For X, Y, Z independently use linear interpolation.
	 * For rotation matrix use spherical interpolation using quaternions and slerp method from Eigen library.
	 */
	Homog_matrix interpolate(double t, const Homog_matrix& other);

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(matrix_m);
	}
};

} // namespace lib
} // namespace mrrocpp

#endif
