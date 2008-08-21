/*!
 * \file ecp_t_spots_recognition_prepare_vector.h
 * \brief Declaration of a class responsible
 * for vector computation, used by Piotr Sakowicz.
 * - class declaration
 * \author vented.baffle
 * \date 21.08.2008
 */

#ifndef _ECP_T_SpotsRecognition_PrepareVector_H_
#define _ECP_T_SpotsRecognition_PrepareVector_H_

#include <stdio.h>

#define PIC_COUNT 10 //zmienic na odczytywanie z pliku

class PrepareVector
{
	double * data;

  public:
	  PrepareVector();
	  PrepareVector(double *);

	  int getPoint(int, double *);

	  //testing (matlab notation)
	  int computeAReal(int, int, int);
	  int computeAReal(int, int);

	  void printData();
	  void printErrors();
};

#endif
