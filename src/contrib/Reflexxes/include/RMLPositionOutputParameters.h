//  ---------------------- Doxygen info ----------------------
//! \file RMLPositionOutputParameters.h
//!
//! \brief
//! Header file for the class RMLPositionOutputParameters
//!
//! \details
//! The class RMLPositionOutputParameters is derived from the class
//! RMLOutputParameters and constitutes a part of the interface for the
//! position-based On-Line Trajectory Generation algorithm.
//!
//! \sa RMLInputParameters
//! \sa RMLPositionInputParameters
//! \sa RMLVelocityOutputParameters
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
//!	\author Torsten Kroeger, <info@reflexxes.com>
//!	
//!
//! \note Copyright (C) 2012 Reflexxes GmbH.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __RMLPositionOutputParameters__
#define __RMLPositionOutputParameters__


#include <RMLOutputParameters.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLPositionOutputParameters
//! 
//! \brief
//! Class for the output parameters of the position-based On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLPositionOutputParameters is derived from the class
//! RMLOutputParameters and constitutes a part of the interface for the
//! position-based On-Line Trajectory Generation algorithm.
//! 
//! \sa ReflexxesAPI
//! \sa RMLOutputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLPositionInputParameters
//  ----------------------------------------------------------
class RMLPositionOutputParameters : public RMLOutputParameters
{

public:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionOutputParameters(const unsigned int DegreesOfFreedom)
//! 
//! \brief
//! Constructor of class RMLPositionOutputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//  ----------------------------------------------------------
    RMLPositionOutputParameters(const unsigned int DegreesOfFreedom) : RMLOutputParameters(DegreesOfFreedom)
    {
    }	


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionOutputParameters(const RMLPositionOutputParameters &OP)
//! 
//! \brief
//! Copy constructor of class RMLPositionOutputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param OP
//! Object to be copied
//  ----------------------------------------------------------
    RMLPositionOutputParameters(const RMLPositionOutputParameters &OP) : RMLOutputParameters(OP)
    {
    }

//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLPositionOutputParameters(void)
//! 
//! \brief
//! Destructor of class RMLPositionOutputParameters
//  ----------------------------------------------------------
    ~RMLPositionOutputParameters(void)
    {
    }


};// class RMLPositionOutputParameters



#endif


