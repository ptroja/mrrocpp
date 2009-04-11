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

} // namespace common
} // namespace ecp
} // namespace mrrocpp
