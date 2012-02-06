//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep1IntermediateProfiles.cpp
//!
//! \brief
//! Implementation file for the Step 1 intermediate velocity profiles of
//! the Type II On-Line Trajectory Generation algorithm
//!
//! \details
//! For further information, please refer to the file
//! TypeIIRMLStep1IntermediateProfiles.h.
//! \n
//! \n
//! <b>GNU Public License</b>
//! \n
//! \n
//! This file is part of the Type II Reflexxes Motion Library.
//! \n\n
//! The Type II Reflexxes Motion Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Type II Reflexxes Motion Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Type II Reflexxes Motion Library. If not, see 
//! <http://www.gnu.org/licenses/>.
//! \n
//! \n
//! \n
//! Reflexxes GmbH\n
//! Sandknoell 7\n
//! D-24805 Hamdorf\n
//! GERMANY\n
//! \n
//! http://www.reflexxes.com\n
//!
//! \date February 2012
//! 
//! \version 1.2
//!
//!	\author Torsten Kroeger, <info@reflexxes.com> \n
//!	
//!
//! \note Copyright (C) 2012 Reflexxes GmbH.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------



#include <TypeIIRMLStep1IntermediateProfiles.h>


using namespace TypeIIRMLMath;


//****************************************************************************
// NegateStep1()

void TypeIIRMLMath::NegateStep1(		double			*ThisCurrentPosition
									,	double			*ThisCurrentVelocity
									,	double			*ThisTargetPosition
									,	double			*ThisTargetVelocity)
{
	*ThisCurrentPosition	=	-(*ThisCurrentPosition	)	;
	*ThisCurrentVelocity	=	-(*ThisCurrentVelocity	)	;
	*ThisTargetPosition		=	-(*ThisTargetPosition	)	;
	*ThisTargetVelocity		=	-(*ThisTargetVelocity	)	;

    return;
}


//****************************************************************************
// VToVMaxStep1()

void TypeIIRMLMath::VToVMaxStep1(		double			*TotalTime
									,	double			*ThisCurrentPosition
									,	double			*ThisCurrentVelocity
									,	const double	&MaxVelocity
									,	const double	&MaxAcceleration)
{
	double	TimeForCurrentStep	=	(*ThisCurrentVelocity - MaxVelocity)
									/ MaxAcceleration;

	*TotalTime				+=	TimeForCurrentStep;
	*ThisCurrentPosition	+=	0.5 * (*ThisCurrentVelocity + MaxVelocity)
								* TimeForCurrentStep;
	*ThisCurrentVelocity	=	MaxVelocity;

    return;
}


//****************************************************************************
// VToZeroStep1()

void TypeIIRMLMath::VToZeroStep1(		double			*TotalTime
									,	double			*ThisCurrentPosition
									,	double			*ThisCurrentVelocity
									,	const double	&MaxAcceleration)
{
	double	TimeForCurrentStep	=	*ThisCurrentVelocity / MaxAcceleration;

	*TotalTime				+=	TimeForCurrentStep;
	*ThisCurrentPosition	+=	0.5 * (*ThisCurrentVelocity) * TimeForCurrentStep;
	*ThisCurrentVelocity	=	0.0;

    return;
}