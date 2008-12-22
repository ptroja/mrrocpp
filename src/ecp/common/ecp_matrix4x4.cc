/*
 * ecp_matrix4x4.cc
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */



#include "ecp/common/ecp_matrix4x4.h"

Matrix4x4::Matrix4x4()
{
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			A[4*i+j] = 0.;
			inv[4*i+j] = 0.;
		}

		b[i] = 0.;
		x[i] = 0.;
	}

	solved = false;
	full = 0;
	type = TYPE_S;
}

Matrix4x4::Matrix4x4(double * Anew, short type_new)
{
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			A[4*i+j] = Anew[4*i+j];
			inv[4*i+j] = 0.;
		}

		b[i] = 0.;
		x[i] = 0.;
	}

	solved = false;
	full = A_LOADED;
	type = type_new;
}

Matrix4x4::Matrix4x4(double * Anew, double * bnew, short type_new)
{
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			A[4*i+j] = Anew[4*i+j];
			inv[4*i+j] = 0.;
		}

		b[i] = bnew[i];
		x[i] = 0.;
	}

	solved = false;
	full = A_LOADED | B_LOADED;
	type = type_new;
}

double * Matrix4x4::getA()
{
	return A;
}


double * Matrix4x4::getb()
{
	return b;
}

double * Matrix4x4::getx()
{
	if(solved)
		return x;
	else if(full == (A_LOADED | B_LOADED))
	{
		solveAxb4x4();
		return x;
	}
	else
		return 0;
}

double * Matrix4x4::getinv()
{
	if(solved)
		return inv;
	else if(full == (A_LOADED | B_LOADED))
	{
		solveAxb4x4();
		return inv;
	}
	else
		return 0;
}

void Matrix4x4::setA(double * Anew, short type_new)
{
	for(int i=0; i<16; i++)
		A[i] = Anew[i];
	full = full | A_LOADED;
	type = type_new;
}

void Matrix4x4::setb(double * bnew)
{
	for(int i=0; i<4; i++)
		b[i] = bnew[i];
	full = full | B_LOADED;
}

double Matrix4x4::det3x3(short x, short y)
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

double Matrix4x4::det4x4()
{
	double det = 0;

	switch(type)
	{
		case TYPE_T:
			return -det3x3(3, 3);
		case TYPE_D:
			for (int i=0, s=-1; i<4; i++, s=-s)
				det += s*det3x3(3, i);
			return det;
		default:
			for (int i=0, s=-1; i<4; i++, s=-s)
				det += s*A[4*i+3]*det3x3(3, i);
			return det;
	}
}

void Matrix4x4::adj4x4(double * out)
{
	for (int i=0, sx=1; i<4; i++, sx=-sx)
	{//4*j+i
		for(int j=0, sy=1; j<4; j++, sy=-sy)
			out[4*j+i] = sx*sy * det3x3(i, j);
	}
}

void Matrix4x4::inv_matrix4x4()
{
	double det = det4x4();
	adj4x4(inv);

	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			inv[4*j+i] /= det;
}

void Matrix4x4::solveAxb4x4()
{
	inv_matrix4x4();

	//x = A^-1 b
	for (int i=0; i<4; i++)
	{
		double temp = 0;
		for (int j=0; j<4; j++)
			temp += inv[4*j+i]*b[j];
		x[i] = temp;
	}
	solved = true;
}

void Matrix4x4::rproduct4x4(double * m)
{
	double temp_m[16];
	for (int i=0; i<4; i++)
	{
		for (int j=0; j<4; j++)
		{
			double temp = 0;
			for(int k=0; k<4; k++)
				temp += A[4*i+k]*m[4*k+j];
			temp_m[4*i+j] = temp;
		}
	}

	for(int i=0; i<16; i++)
		A[i] = temp_m[i];
}

void Matrix4x4::lproduct4x4(double * m)
{
	double temp_m[16];
	for (int i=0; i<4; i++)
	{
		for (int j=0; j<4; j++)
		{
			double temp = 0;
			for(int k=0; k<4; k++)
				temp += m[4*i+k]*A[4*k+j];
			temp_m[4*i+j] = temp;
		}
	}

	for(int i=0; i<16; i++)
		A[i] = temp_m[i];
}

void Matrix4x4::product4x1(double * vec)
{
	double temp_v[4];
	for (int j=0; j<4; j++)
	{
		double temp = 0;
		for(int k=0; k<4; k++)
			temp += A[4*j+k]*vec[k];
		temp_v[j] = temp;
	}

	for(int i=0; i<4; i++)
		vec[i] = temp_v[i];
}

void Matrix4x4::product1x4(double * vec)
{
	double temp_v[4];
	for (int j=0; j<4; j++)
	{
		double temp = 0;
		for(int k=0; k<4; k++)
			temp += A[4*k+j]*vec[k];
		temp_v[j] = temp;
	}

	for(int i=0; i<4; i++)
		vec[i] = temp_v[i];
}
