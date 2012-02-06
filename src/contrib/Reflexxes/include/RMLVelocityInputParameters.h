//  ---------------------- Doxygen info ----------------------
//! \file RMLVelocityInputParameters.h
//!
//! \brief
//! Header file for the class RMLVelocityInputParameters
//!
//! \details
//! The class RMLVelocityInputParameters is derived from the class
//! RMLInputParameters and constitutes a part of the interface for the
//! velocity-based On-Line Trajectory Generation algorithm.
//!
//! \sa RMLInputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLPositionInputParameters
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


#ifndef __RMLVelocityInputParameters__
#define __RMLVelocityInputParameters__


#include <RMLInputParameters.h>
#include <math.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLVelocityInputParameters
//! 
//! \brief
//! Class for the input parameters of the velocity-based On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLVelocityInputParameters is derived from the class
//! RMLInputParameters and constitutes a part of the interface for the
//! velocity-based On-Line Trajectory Generation algorithm.\n\n
//! 
//! A detailed description can be found at \ref page_InputValues.
//! 
//! \sa ReflexxesAPI
//! \sa RMLInputParameters
//! \sa RMLPositionInputParameters
//! \sa RMLVelocityOutputParameters
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
class RMLVelocityInputParameters : public RMLInputParameters
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityInputParameters(const unsigned int DegreesOfFreedom)
//! 
//! \brief
//! Constructor of class RMLVelocityInputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//  ----------------------------------------------------------
    RMLVelocityInputParameters(const unsigned int DegreesOfFreedom) : RMLInputParameters(DegreesOfFreedom)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityInputParameters(const RMLVelocityInputParameters& IP)
//! 
//! \brief
//! Copy constructor of class RMLVelocityInputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param IP
//! Object to be copied
//  ----------------------------------------------------------
    RMLVelocityInputParameters(const RMLVelocityInputParameters &IP) : RMLInputParameters(IP)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLVelocityInputParameters(void)
//! 
//! \brief
//! Destructor of class RMLVelocityInputParameters
//  ----------------------------------------------------------
    ~RMLVelocityInputParameters(void)
    {
    }
    
    
//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityInputParameters &operator = (const RMLVelocityInputParameters &IP)
//! 
//! \brief
//! Copy operator
//! 
//! \param IP
//! RMLVelocityInputParameters object to be copied
//  ----------------------------------------------------------	
    inline RMLVelocityInputParameters &operator = (const RMLVelocityInputParameters &IP)
    {
        RMLInputParameters::operator=(IP);

        return(*this);
    }
        


// #############################################################################	
    

//  ---------------------- Doxygen info ----------------------
//! \fn bool CheckForValidity(void) const
//! 
//! \brief
//! Checks the input parameters for validity
//! 
//! \details
//! \copydetails RMLPositionInputParameters::CheckForValidity()
//  ----------------------------------------------------------
    bool CheckForValidity(void) const
    {
        unsigned int		i							=	0;
    
        double				MinimumOrderOfMagnitude		=	0.0
                        ,	MaximumOrderOfMagnitude		=	0.0;
                        
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->SelectionVector->VecData)[i])
            {
                if (
#ifdef RMLTYPEIV 					
					((this->MaxAccelerationVector->VecData)[i]	>=	(this->MaxJerkVector->VecData)					[i]	)
                    &&	
#endif
						((this->MaxAccelerationVector->VecData)[i]	>=	fabs((this->TargetVelocityVector->VecData)		[i]))
                    &&	((this->MaxAccelerationVector->VecData)[i]	>=	fabs((this->CurrentPositionVector->VecData)		[i]))
                    &&	((this->MaxAccelerationVector->VecData)[i]	>=	fabs((this->CurrentVelocityVector->VecData)		[i]))
#ifdef RMLTYPEIV 
                    &&	((this->MaxAccelerationVector->VecData)[i]	>=	fabs((this->CurrentAccelerationVector->VecData)	[i]))
#endif					
					)
                {
                    MaximumOrderOfMagnitude	=	(this->MaxAccelerationVector->VecData)[i];
                }
                else
                {
#ifdef RMLTYPEIV 
                    if (	((this->MaxJerkVector->VecData)[i]	>=	fabs((this->TargetVelocityVector->VecData)		[i]))
                        &&	((this->MaxJerkVector->VecData)[i]	>=	fabs((this->CurrentPositionVector->VecData)		[i]))
                        &&	((this->MaxJerkVector->VecData)[i]	>=	fabs((this->CurrentVelocityVector->VecData)		[i]))
                        &&	((this->MaxJerkVector->VecData)[i]	>=	fabs((this->CurrentAccelerationVector->VecData)	[i]))

						)
                    {
                        MaximumOrderOfMagnitude	=	(this->MaxJerkVector->VecData)[i];
                    }
                    else
#endif
                    {
                        if (	(fabs((this->TargetVelocityVector->VecData)[i])	>=	fabs((this->CurrentPositionVector->VecData)		[i]))
                            &&	(fabs((this->TargetVelocityVector->VecData)[i])	>=	fabs((this->CurrentVelocityVector->VecData)		[i]))
#ifdef RMLTYPEIV 
                            &&	(fabs((this->TargetVelocityVector->VecData)[i])	>=	fabs((this->CurrentAccelerationVector->VecData)	[i]))
#endif
							)
                        {
                            MaximumOrderOfMagnitude	=	fabs((this->TargetVelocityVector->VecData)[i]);
                        }
                        else
                        {
                            if (	(fabs((this->CurrentPositionVector->VecData)[i])	>=	fabs((this->CurrentVelocityVector->VecData)		[i]))
#ifdef RMLTYPEIV 
                                &&	(fabs((this->CurrentPositionVector->VecData)[i])	>=	fabs((this->CurrentAccelerationVector->VecData)	[i]))
#endif
								)
                            {
                                MaximumOrderOfMagnitude	=	fabs((this->CurrentPositionVector->VecData)[i]);
                            }
#ifdef RMLTYPEIV 
                            else
                            {
                                MaximumOrderOfMagnitude	=	fabs((this->CurrentAccelerationVector->VecData)[i]);
                            }
#endif
                        }
                    }		
                }				

                if ((this->MaxAccelerationVector->VecData)[i]	<=	(this->MaxJerkVector->VecData)[i])
                {
                    MinimumOrderOfMagnitude	=	(this->MaxAccelerationVector->VecData)[i];
                }
                else
                {
                    MinimumOrderOfMagnitude	=	(this->MaxJerkVector->VecData)[i];
                }										
                
                // The value of MinimumOrderOfMagnitude is greater than
                // zero:				
                if (	(MaximumOrderOfMagnitude / MinimumOrderOfMagnitude)
                    >	(double)pow((float)10, (int)(RMLVelocityInputParameters::MAXIMUM_MAGNITUDE_RANGE)))
                {
                    return(false);
                }
            }
        }
        return(true);		
    }
    
    
//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//! 
//! \brief
//! \copybrief RMLInputParameters::Echo()
//! 
//! \details
//! \copydetails RMLInputParameters::Echo()
//  ----------------------------------------------------------
    void Echo(FILE* FileHandler = stdout) const
    {
        RMLInputParameters::Echo(FileHandler);
        return;
    }	

protected:


    enum
    {
//  ---------------------- Doxygen info ----------------------
//! \brief
//! Specifies the maximum allowed range for the orders of magnitude of
//! the input values.
//  ----------------------------------------------------------	
        MAXIMUM_MAGNITUDE_RANGE	=	10
    };



};// class RMLVelocityInputParameters



#endif


