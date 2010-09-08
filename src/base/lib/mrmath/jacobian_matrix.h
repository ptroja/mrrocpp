/**
 * \file jacobian_matrix.h
 *
 * \brief Jacobian 6x6 matrix class
 *
 * \author Anna Sibilska <asibilsk@elka.pw.edu.pl>
 */

#ifndef __JACOBIAN_MATRIX_H
#define __JACOBIAN_MATRIX_H

#include "base/lib/impconst.h"	// frame_tab

namespace mrrocpp {
namespace lib {

/**
 * Jacobian 6x6 matrix class
 *
 * @todo Rewrite with Eigen matrix
 */
class Jacobian_matrix
{
private:
	//! Matrix data place-holder
	double matrix[6][6];

public:
	/**
	 * Constructor
	 */
	Jacobian_matrix();

	/**
	 Wyznaczenie jakobianu manipulatora dla zadanej aktualnej konfiguracji - wzory analityczne

	 @param[in] local_current_joints - obecne wartosci wspolrzednych wewnetrznych robota (kolejno q0, q1, q2, ...)
	 zadane w postaci wektora Ft_v_vector
	 */
	void irp6_6dof_equations(const Xyz_Angle_Axis_vector & w); //Wzory na jakobian dla Irp-6 o 6 stopniach swobody

	/**
	 Wyznaczenie odwrotnosci jakobianu manipulatora irp6 o 6 stopniach swobody dla zadanej
	 aktualnej konfiguracji - wzory analityczne. (Wzory bez uwzglednienia narzedzia)

	 @param[in] local_current_joints obecne wartosci wspolrzednych wewnetrznych robota
	 (kolejno q0, q1, q2, ...) zadane w postaci wektora Ft_v_vector
	 */
	void irp6_6dof_inverse_equations(const Xyz_Angle_Axis_vector & w); //Wzory na odwrotnosc jakobianu dla Irp-6 o 6 stopniach swobody

	/**
	 * Wyliczenie wartosci wyznacznika jakobianu manipulatora irp6 o 6 stopniach
	 * swobody dla zadanej aktualnej konfiguracji - wzory analityczne.
	 * (Wzory bez uwzglednienia narzedzia)
	 *
	 * @param[in] local_current_joints obecne wartosci wspolrzednych wewnetrznych robota (kolejno q0, q1, q2, ...) zadane w postaci wektora Ft_v_vector
	 * @return wartosc wyznacznika macierzy jakobianowej
	 */
	double irp6_6dof_determinant(const Xyz_Angle_Axis_vector & w); //Wzory na wyznacznik jaokbianu dla Irp-6 o 6 stopniach swobody

	/**
	 * Transpose
	 */
	void transpose();

	/**
	 * Print to console
	 */
	void print();

	/**
	 * Export to a C-style two-dimensional-array
	 *
	 * @param[out] array place to export values
	 */
	void to_table(double array[6][6]) const;

	/**
	 * Solve AX=Y (A, Y - given) system of equations with Gaussian_elimination
	 */
	Xyz_Angle_Axis_vector jacobian_inverse_gauss(const Xyz_Angle_Axis_vector & dist);

	/**
	 * Overloaded matrix multiplicity operator
	 *
	 * @param[in] multiplicand matrix
	 */
	Xyz_Angle_Axis_vector operator*(const Xyz_Angle_Axis_vector & w) const;
};

} // namespace lib
} // namespace mrrocpp

#endif
