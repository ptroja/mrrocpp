//  ---------------------- Doxygen info ----------------------
//! \file RMLVelocityOutputParameters.h
//!
//! \brief
//! Header file for the class RMLVelocityOutputParameters
//!
//! \details
//! The class RMLVelocityOutputParameters is derived from the class
//! RMLOutputParameters and constitutes a part of the interface for the
//! velocity-based On-Line Trajectory Generation algorithm.
//!
//! \sa RMLInputParameters
//! \sa RMLVelocityInputParameters
//! \sa RMLPositionOutputParameters
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


#ifndef __RMLVelocityOutputParameters__
#define __RMLVelocityOutputParameters__


#include <RMLOutputParameters.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLVelocityOutputParameters
//! 
//! \brief
//! Class for the output parameters of the velocity-based On-Line
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLVelocityOutputParameters is derived from the class
//! RMLOutputParameters and constitutes a part of the interface for the
//! velocity-based On-Line Trajectory Generation algorithm.
//! 
//! \sa ReflexxesAPI
//! \sa RMLOutputParameters
//! \sa RMLPositionOutputParameters
//! \sa RMLVelocityInputParameters
//  ----------------------------------------------------------
class RMLVelocityOutputParameters : public RMLOutputParameters
{

public:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityOutputParameters(const unsigned int DegreesOfFreedom)
//! 
//! \brief
//! Constructor of class RMLVelocityOutputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//  ----------------------------------------------------------
    RMLVelocityOutputParameters(const unsigned int DegreesOfFreedom) : RMLOutputParameters(DegreesOfFreedom)
    {
        this->PositionValuesAtTargetVelocity	=	new RMLDoubleVector	(DegreesOfFreedom)	;			

        memset(this->PositionValuesAtTargetVelocity->VecData	,	0x0	,		DegreesOfFreedom * sizeof(double))	;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityOutputParameters(const RMLVelocityOutputParameters &OP)
//! 
//! \brief
//! Copy constructor of class RMLVelocityOutputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param OP
//! Object to be copied
//  ----------------------------------------------------------
    RMLVelocityOutputParameters(const RMLVelocityOutputParameters &OP) : RMLOutputParameters(OP)
    {
        this->PositionValuesAtTargetVelocity		=	new RMLDoubleVector	((OP.PositionValuesAtTargetVelocity)->GetVecDim())	;

        *(this->PositionValuesAtTargetVelocity)		=	*(OP.GetReferenceOfThePositionValuesAtTargetVelocityObject())			;
    }

//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLVelocityOutputParameters(void)
//! 
//! \brief
//! Destructor of class RMLVelocityOutputParameters
//  ----------------------------------------------------------
    ~RMLVelocityOutputParameters(void)
    {	
        delete	this->PositionValuesAtTargetVelocity		;
        
        this->PositionValuesAtTargetVelocity	=	NULL	;		
    }
    

//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityOutputParameters &operator = (const RMLVelocityOutputParameters &OP)
//! 
//! \brief
//! Copy operator
//! 
//! \param OP
//! RMLVelocityOutputParameters object to be copied
//  ----------------------------------------------------------	
    RMLVelocityOutputParameters &operator = (const RMLVelocityOutputParameters &OP)
    {
        RMLOutputParameters::operator=(OP);
        
        *(this->PositionValuesAtTargetVelocity)	=	*(OP.GetReferenceOfThePositionValuesAtTargetVelocityObject())	;

        return(*this);
    }
    

// #############################################################################


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetPositionValuesAtTargetVelocity(RMLDoubleVector *OutputVector) const
//! 
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! position values for each degree of freedom, at which the 
//! desired target velocity \f$\ _{k}V_{i}^{\,trgt} \f$ is reached, to the
//! \c RMLDoubleVector object referred to by \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be 
//! copied
//! 
//! \sa GetPositionValuesAtTargetVelocity(double *OutputVector, const unsigned int &SizeInBytes) const
//! \sa GetPositionValuesAtTargetVelocityElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetReferenceOfThePositionValuesAtTargetVelocityObject(void) const
//! \sa GetReferenceOfThePositionValuesAtTargetVelocityData(void) const
//  ----------------------------------------------------------
    inline void GetPositionValuesAtTargetVelocity(RMLDoubleVector *OutputVector) const
    {
        *OutputVector	=	*(this->PositionValuesAtTargetVelocity);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetPositionValuesAtTargetVelocity(double *OutputVector, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies the array of \c double values representing the position values
//! for each degree of freedom, at which the desired target velocity
//! \f$\ _{k}V_{i}^{\,trgt} \f$ is reached, to the memory pointed to by
//! \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an array of \c double values, to which the data will be 
//! copied
//! 
//! \param SizeInBytes
//! The size of available memory at the location pointed to by 
//! \c OutputVector. To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//! 
//! \sa GetPositionValuesAtTargetVelocity(RMLDoubleVector *OutputVector) const
//! \sa GetPositionValuesAtTargetVelocityElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetReferenceOfThePositionValuesAtTargetVelocityObject(void) const
//! \sa GetReferenceOfThePositionValuesAtTargetVelocityData(void) const
//  ----------------------------------------------------------
    inline void GetPositionValuesAtTargetVelocity(		double				*OutputVector
                                                    ,	const unsigned int	&SizeInBytes) const
    {
        memcpy(		(void*)OutputVector
                ,	(void*)this->PositionValuesAtTargetVelocity->VecData
                ,	SizeInBytes	);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetPositionValuesAtTargetVelocityElement(double *OutputValue, const unsigned int &Index) const
//! 
//! \brief
//! Copies one element of the position values for each degree of freedom,
//! at which the desired target velocity \f$\ _{k}V_{i}^{\,trgt} \f$
//! is reached, to the memory pointed to by \c OutputValue
//! 
//! \param OutputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//! 
//! \param Index
//! Specifies the desired element of the vector. The element numbering 
//! starts with \em 0 (zero). If this value is greater the number 
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c OutputValue.
//! 
//! \sa GetPositionValuesAtTargetVelocity(RMLDoubleVector *OutputVector) const
//! \sa GetPositionValuesAtTargetVelocityElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetPositionValuesAtTargetVelocityElement(const unsigned int &Index) const
//! \sa GetReferenceOfThePositionValuesAtTargetVelocityObject(void) const
//! \sa GetReferenceOfThePositionValuesAtTargetVelocityData(void) const
//  ----------------------------------------------------------
    inline void GetPositionValuesAtTargetVelocityElement(		double				*OutputValue
                                                            ,	const unsigned int	&Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->PositionValuesAtTargetVelocity->GetVecDim() ) )
        {
            *OutputValue	=	0.0;
        }
        else
        {	
            *OutputValue	=	(*this->PositionValuesAtTargetVelocity)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetPositionValuesAtTargetVelocityElement(const unsigned int &Index) const
//! 
//! \brief
//! Returns one single element of the position values
//! for each degree of freedom, at which the desired target velocity
//! \f$\ _{k}V_{i}^{\,trgt} \f$ is reached
//! 
//! \param Index
//! Specifies the desired element of the vector. The index of the first 
//! vector element is \em 0 (zero). If the value of \c Index value is 
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c OutputValue.
//! 
//! \sa GetPositionValuesAtTargetVelocityElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetPositionValuesAtTargetVelocityElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->PositionValuesAtTargetVelocity->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {		
            return( (*this->PositionValuesAtTargetVelocity)[Index] );
        }		
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline RMLDoubleVector* GetReferenceOfThePositionValuesAtTargetVelocityObject(void) const
//! 
//! \brief
//! Returns a pointer to an \c RMLDoubleVector object that contains the 
//! position values for each degree of freedom, at which the desired
//! target velocity \f$\ _{k}V_{i}^{\,trgt} \f$ is reached
//! 
//! \details
//! This method may be useful to directly access the RMLDoubleVector object.
//! 
//! \sa GetReferenceOfThePositionValuesAtTargetVelocityData(void) const
//! \sa RMLVector
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline RMLDoubleVector* GetReferenceOfThePositionValuesAtTargetVelocityObject(void) const
    {
        return(this->PositionValuesAtTargetVelocity);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double* GetReferenceOfThePositionValuesAtTargetVelocityData(void) const
//! 
//! \brief
//! Returns a pointer to an array of \c double values that contain the 
//! position values for each degree of freedom, at which the desired
//! target velocity \f$\ _{k}V_{i}^{\,trgt} \f$ is reached
//! 
//! \details
//! The returned value is a pointer to the memory area that contains the 
//! pure content data of an \c RMLDoubleVector object. Using this pointer,
//! the content of this input vector can be directly accessed and any
//! additional copy operations will be completely avoided to save CPU time.
//! 
//! \sa GetReferenceOfThePositionValuesAtTargetVelocityObject(void) const
//! \sa RMLVector
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline double* GetReferenceOfThePositionValuesAtTargetVelocityData(void) const
    {
        return((double*)this->PositionValuesAtTargetVelocity->VecData);
    }


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *PositionValuesAtTargetVelocity
//! 
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the position values at the instants the desired target velocity 
//! is reached
//!
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetPositionValuesAtTargetVelocity(RMLDoubleVector *OutputVector) const\n\n 
//!  - GetPositionValuesAtTargetVelocity(double *OutputVector, const unsigned int &SizeInBytes) const\n\n 
//!  - GetPositionValuesAtTargetVelocityElement(double *OutputValue, const unsigned int &Index) const\n\n 
//!  - GetPositionValuesAtTargetVelocityElement(const unsigned int &Index) const\n\n 
//!  - GetReferenceOfThePositionValuesAtTargetVelocityObject(void) const\n\n 
//!  - GetReferenceOfThePositionValuesAtTargetVelocityData(void) const\n\n 
//  ----------------------------------------------------------
    RMLDoubleVector			*PositionValuesAtTargetVelocity;


};// class RMLVelocityOutputParameters



#endif


