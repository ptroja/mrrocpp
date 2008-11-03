/////////////////////////////////////////////////////////
// Name: KociembaException.h
// Implements: KociembaException
/////////////////////////////////////////////////////////

#include "vsp/rcs/Kociemba/KociembaException.h"

#include <string.h>


const char *KociembaException::TYPE = "Kociemba";

////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, COPY and private helpers for them
////////////////////////////////////////////////////////////////////////////////

KociembaException::KociembaException(int error)
    : Exception(error)
{
}

KociembaException::KociembaException(const KociembaException& exp)
    : Exception(exp.iError)
{
}

KociembaException::~KociembaException()
{
}

Exception* KociembaException::Clone()
{
    return new KociembaException(*this);
}


////////////////////////////////////////////////////////////////////////////////
// ERROR TEXTS AND ACCESS TO THEM
////////////////////////////////////////////////////////////////////////////////

char* KociembaException::ErrorText[] = 
  {  
	  (char*)"",
	  (char*)"Phase 2 cube is invalid.",
	  (char*)"Move is not allowed on phase 2 cube.",
	  (char*)"Invalid coordinate number for Kociemba Cube move table.",
	  (char*)"Invalid coordinate numbers for Kociemba Cube prunung table.",
	  (char*)"Solver accepts only CubieCube or KociembaPhase1Cube as input.",
	  (char*)"Solver accepts only CubieCube or KociembaPhase2Cube as input.",
	  (char*)"Solver accepts only CubieCube, KociembaPhase1Cube or KociembaPhase2Cube as input.",
	  (char*)"Next solution can be found only for not optimal solutions.",
	  (char*)"Solution parsing failed on invalid string format", 
	  (char*)"Solution parsing failed on invalid move in phase1", 
	  (char*)"Solution parsing failed on invalid turn in phase1", 
	  (char*)"Solution parsing failed on invalid string format in phase1", 
	  (char*)"Solution parsing failed on invalid move in phase2", 
	  (char*)"Solution parsing failed on invalid turn in phase2", 
	  (char*)"Solution parsing failed on invalid string format in phase2", 
  };

const char* KociembaException::GetErrorText()
{
    if (iError >= 0 && iError < ERR_NUMBER)
        return ErrorText[iError];
    else 
        return "??";
}
