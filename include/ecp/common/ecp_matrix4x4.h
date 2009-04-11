/*
 * ecp_matrix4x4.h
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */

#ifndef ECP_MATRIX_4x4_H_
#define ECP_MATRIX_4x4_H_

#include <iostream>
#include <cmath>

namespace mrrocpp {
namespace ecp {
namespace common {

class T_MatrixManip
{
	double A[16];

	double det3x3(short, short);
	double det4x4();
	void adj4x4(double[12]);

  public:
	T_MatrixManip(double[12]);

	void inv_matrix4x4(double[12]);
	void multiply_r_matrix4x4(double[12], double[12]);
	void multiply_l_matrix4x4(double[12], double[12]);
};


} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_MATRIX4X4_H_ */
