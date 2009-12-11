/////////////////////////////////////////////////////////
// Name: KorfException.h
// Implements: KorfException
/////////////////////////////////////////////////////////

#include "vsp/rcs/Korf/KorfException.h"

#include <string.h>


const char *KorfException::TYPE = "Korf";

////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, COPY and private helpers for them
////////////////////////////////////////////////////////////////////////////////

KorfException::KorfException(int error)
    : Exception(error)
{
}

KorfException::KorfException(const KorfException& exp)
    : Exception(exp.iError)
{
}

KorfException::~KorfException()
{
}

Exception* KorfException::Clone()
{
    return new KorfException(*this);
}


////////////////////////////////////////////////////////////////////////////////
// ERROR TEXTS AND ACCESS TO THEM
////////////////////////////////////////////////////////////////////////////////

const char* KorfException::ErrorText[] = 
  {  
	  "",
	  "Solver accepts only CubieCube or KorfCube as input.",
  };

const char* KorfException::GetErrorText()
{
    if (iError > 0 && iError < ERR_NUMBER)
        return ErrorText[iError];
    else 
        return "??";
}
