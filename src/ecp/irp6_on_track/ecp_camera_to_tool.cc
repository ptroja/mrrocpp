/*
 * ecp_camera_to_tool.cc
 *
 *  Created on: Apr 8, 2009
 *      Author: ghard
 */

#include "ecp/irp6_on_track/ecp_camera_to_tool.h"

namespace mrrocpp {
namespace ecp {
namespace common {

using namespace std;

CameraToTool::CameraToTool(double dn, double hn, double an)
{
	d = dn;
	h = hn;
	a = an;

	fillTPE();
}

void CameraToTool::fillTPE()
{
	double b = sqrt(2)/2;

	tpe[0] = tpe[1] = tpe[6] = tpe[7] = tpe[10] = 0;
	tpe[2] = 1;
	tpe[3] = h;
	tpe[4] = tpe[8] = tpe[9] = -b;
	tpe[5] = b;
	tpe[11] = d;

}

double CameraToTool::computeTCE(double vecs[12], double ret[12])
{
	double vec1[3], vec2[3], vec3[3], vec4[3];

	for(int i=0; i<3; i++)
	{
		vec1[i] = vecs[i];
		vec2[i] = vecs[3+i];
		vec3[i] = vecs[6+i];
		vec4[i] = vecs[9+i];
	}

	return computeTCE(vec1, vec2, vec3, vec4, ret);
}

double CameraToTool::computeTCE(double vec1[3], double vec2[3], double vec3[3], double vec4[3], double ret[12])
{
	double norm2_TPC = computeTPC(vec1, vec2, vec3, vec4, ret);
	double tcp[12];

	T_MatrixManip Tpc_mm(ret);
	Tpc_mm.inv_matrix4x4(tcp);

	T_MatrixManip Tcp_mm(tcp);
	//TCE = TPE*TCP
	Tcp_mm.multiply_l_matrix4x4(tpe, ret);

	return norm2_TPC;
}

double CameraToTool::computeTPC(double vec1[3], double vec2[3], double vec3[3], double vec4[3], double ret[12])
{
	double m1234[3], ri1[3], ri2[3], r3[3];
	double b = a*sqrt(2);

	for(int i=0; i<3; i++)
	{
		m1234[i] = 0.25*(vec1[i]+vec2[i]+vec3[i]+vec4[i]);
		ri1[i] = (vec1[i]-vec3[i])/b;
		ri2[i] = (vec2[i]-vec4[i])/b;
	}

	vec_prod(ri1, ri2, r3);

	for(int i=0; i<3; i++)
	{
		ret[4*i] = ri1[i];
		ret[4*i+1] = ri2[i];
		ret[4*i+2] = r3[i];
		ret[4*i+3]=m1234[i];
	}

	return norm2m(ret);
}

double CameraToTool::norm2v(double ret[3])
{
	double sum = 0;

	for (int i=0; i<3; i++)
		sum += ret[i]*ret[i];

	return sqrt(sum);
}

double CameraToTool::norm2m(double ret[12])
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

void CameraToTool::vec_prod(double r1[3], double r2[3], double r3[3])
{
	r3[0] = r1[1]*r2[2] - r2[1]*r1[2];
	r3[1] = r1[2]*r2[0] - r2[2]*r1[0];
	r3[2] = r1[0]*r2[1] - r2[0]*r1[1];
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp

