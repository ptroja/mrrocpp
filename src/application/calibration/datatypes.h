/*
 * datatypes.h
 *
 *  Created on: May 17, 2010
 *      Author: ptroja
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

// struktura z informacja czy znaleziono szachownice
typedef struct _chessboard
{
	int frame_number;
	float transformation_matrix[12];
	bool found;
	bool calibrated;
} chessboard_t;

// structure used to send to fradia data needed for eih calibration
typedef struct _eihcalibration
{
	int frame_number;
	float transformation_matrix[12];
} eihcalibration_t;

#endif /* DATATYPES_H_ */
