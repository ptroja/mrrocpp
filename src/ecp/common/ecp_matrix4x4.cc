/*
 * ecp_matrix4x4.cc
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */

#include "ecp/common/ecp_matrix4x4.h"

namespace mrrocpp {
namespace ecp {
namespace common {

using namespace std;

T_MatrixManip::T_MatrixManip(double m[12])
{
	for(int i=0; i<12; i++)
		A[i] = m[i];

	A[12] = A[13] = A[14] = 0;
	A[15] = 1;
}

double T_MatrixManip::det3x3(short x, short y)
{
	short x_map[3];
	short y_map[3];

	for (int i=0, act_x=0, act_y=0; i<4; i++)
	{
		if(i!=x)
			x_map[act_x++] = i;
		if(i!=y)
			y_map[act_y++] = i;
	}

	return A[x_map[0]+4*y_map[0]] * A[x_map[1]+4*y_map[1]] * A[x_map[2]+4*y_map[2]] + A[x_map[1]+4*y_map[0]] * A[x_map[2]+4*y_map[1]] * A[x_map[0]+4*y_map[2]] + A[x_map[2]+4*y_map[0]] * A[x_map[0]+4*y_map[1]] * A[x_map[1]+4*y_map[2]] - A[x_map[0]+4*y_map[2]] * A[x_map[1]+4*y_map[1]] * A[x_map[2]+4*y_map[0]] - A[x_map[0]+4*y_map[1]] * A[x_map[1]+4*y_map[0]] * A[x_map[2]+4*y_map[2]] - A[x_map[0]+4*y_map[0]] * A[x_map[1]+4*y_map[2]] * A[x_map[2]+4*y_map[1]];
}

double T_MatrixManip::det4x4()
{
	return det3x3(3, 3);;
}

void T_MatrixManip::adj4x4(double out[12])
{
	for (int i=0, sx=1; i<3; i++, sx=-sx)
	{//4*j+i
		for(int j=0, sy=1; j<4; j++, sy=-sy)
			out[4*i+j] = sx*sy * det3x3(i, j);
	}
}

void T_MatrixManip::inv_matrix4x4(double inv[12])
{
	double det = det4x4();
	adj4x4(inv);

	for (int i=0; i<4; i++)
		for (int j=0; j<3; j++)
			inv[4*j+i] /= det;
}

void T_MatrixManip::multiply_r_matrix4x4(double B[12], double ret[12])
{
	for (int i=0; i<4; i++)
		for (int j=0; j<3; j++)
		{
			ret[4*j+i] = 0;

			for (int k=0; k<3; k++)
				ret[4*j+i] += A[4*j+k]*B[4*k+i];

			if(i==3)
				ret[4*j+i] += A[4*j+3];
		}
}

void T_MatrixManip::multiply_l_matrix4x4(double B[12], double ret[12])
{
	for (int i=0; i<4; i++)
		for (int j=0; j<3; j++)
	{
		ret[4*j+i] = 0;

		for (int k=0; k<3; k++)
			ret[4*j+i] += B[4*j+k]*A[4*k+i];

		if(i==3)
			ret[4*j+i] += B[4*j+3];
	}
}

double T_MatrixManip::norm2m(double ret[12])
{
	double norm = -1.;

	for (int i=0; i<3; i++)
	{
		double vec[3];
		vec[0] = ret[4*i];
		vec[1] = ret[4*i+1];
		vec[2] = ret[4*i+2];

		double temp = norm2v(vec);
		if (temp > norm)
			norm = temp;
	}

	return norm;
}

double T_MatrixManip::norm2v(double ret[3])
{
	double sum = 0;

	for (int i=0; i<3; i++)
		sum += ret[i]*ret[i];

	return sqrt(sum);
}

Spots_Data::Spots_Data()
{
	tcg = new vector<matrix12_t>;
	vec_cam = new vector<vector4_t>;
	vec_ground = new vector<vector4_t>;

	count = 0;
}

Spots_Data::~Spots_Data()
{
	delete tcg;
	delete vec_cam;
	delete vec_ground;
}

void Spots_Data::add_tcg(double tcg_rec[12])
{
	matrix12_t m(tcg_rec, tcg_rec+12);
	tcg->push_back(m);
	count++;


	cout << "tcg = [" << endl;
	cout << m[0] << "  " << m[1] << "  " << m[2] << "  " << m[3] << endl;
	cout << m[4] << "  " << m[5] << "  " << m[6] << "  " << m[7] << endl;
	cout << m[8] << "  " << m[9] << "  " << m[10] << "  " << m[11] << endl;
	cout << "0 0 0 1];" << endl;
}

void Spots_Data::add_vec_cam(double vec_cam_rec[4])
{
	vector4_t v(vec_cam_rec, vec_cam_rec+4);
	tcg->push_back(v);


	cout << "vec_cam = [" << endl;
	cout << v[0] << "  " << v[1] << "  " << v[2] << "  " << v[3] << endl;
	cout << "]';" << endl;
}

void Spots_Data::add_vec_ground(double vec_ground_rec[4])
{
	vector4_t v(vec_ground_rec, vec_ground_rec+4);
	tcg->push_back(v);


	cout << "vec_ground = [" << endl;
	cout << v[0] << "  " << v[1] << "  " << v[2] << "  " << v[3] << endl;
	cout << "]';" << endl;
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp
