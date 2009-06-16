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
#include <vector>

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {

typedef vector<double> matrix12_t;
typedef vector<double> vector4_t;

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

	static double norm2m(double[12]);
	static double norm2v(double[3]);
};

class Spots_Data
{
	vector<matrix12_t> * tcg;
	vector<vector4_t> * vec_cam;
	vector<vector4_t> * vec_ground;

	int count;

  public:
	Spots_Data();
	~Spots_Data();

	void add_tcg(double[12]);
	void add_vec_cam(double[4]);
	void add_vec_ground(double[4]);
};


} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_MATRIX4X4_H_ */
