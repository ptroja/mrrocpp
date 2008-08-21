/*!
 * \file ecp_t_spots_recognition_prepare_vector.cc
 * \brief Declaration of a class responsible
 * for vector computation, used by Piotr Sakowicz.
 * - methods definitions
 * \author vented.baffle
 * \date 21.08.2008
 */

#include "ecp/common/ecp_t_spots_recognition_prepare_vector.h"

PrepareVector::PrepareVector()
{
	data = 0;
}

PrepareVector::PrepareVector(double * in)
{
	for (int i=0; i<13*PIC_COUNT; i++)
		data[i] = in[i];
}

//vec must be initialized (memory allocated), n = 1:4
int PrepareVector::getPoint(int n, double * vec)
{
	if (n<1 || n>4)
		return -1;

	for (int j=0; j<3; j++)
		vec[j]=0;

	for (int i=0; i<PIC_COUNT; i++)
		for (int j=0; j<3; j++)
		    vec[j] += data[13*i + 3*(n-1) + j];

	for (int j=0; j<3; j++)
		vec[j] /= PIC_COUNT;

	return 3;

}

//testing (matlab notation)
int PrepareVector::computeAReal(int pic, int p1, int p2)
{
	//TODO
	return 0;
}

int PrepareVector::computeAReal(int pic, int d)
{
	switch(d)
	{
	  case 1:
		  return computeAReal(pic, 1, 2);
	  case 2:
		  return computeAReal(pic, 2, 3);
	  case 3:
		  return computeAReal(pic, 3, 4);
	  case 4:
		  return computeAReal(pic, 4, 1);
	  default:
		  return -1;
	}
}

void PrepareVector::printData()
{
	for (int i=0; i<PIC_COUNT; i++)
		for (int j=0; j<4; j++)
		{
			printf("p%d(%d,:) = [%f, %f, %f]\n, ", j+1, i+1, data[13*i + 3*j], data[13*i + 3*j+1], data[13*i + 3*j+2]);
		}
}
void PrepareVector::printErrors()
{
	//TODO
}
