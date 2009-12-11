/////////////////////////////////////////////////////////
// Name: CubeSolutionException.h
// Implements: CubeSolutionException
/////////////////////////////////////////////////////////

#include "vsp/rcs/Cube/CubeSolutionException.h"
#include "vsp/rcs/Cube/CubeSolution.h"

#include <string.h>


const char *CubeSolutionException::TYPE = "CubeSolution";

////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, COPY and private helpers for them
////////////////////////////////////////////////////////////////////////////////

CubeSolutionException::CubeSolutionException(const char *sSol, int val, int error)
    : Exception(error), sSol(NULL), iValue(val)
{
    if (sSol != NULL) 
    {
        this->sSol = new char[strlen(sSol)+1];
        strncpy(this->sSol, sSol, strlen(sSol)+1);
    }
}

CubeSolutionException::CubeSolutionException(const CubeSolutionException& exp)
    : Exception(exp.iError), sSol(NULL), iValue(exp.iValue)
{
    if (exp.sSol != NULL) 
    {
        this->sSol = new char[strlen(exp.sSol)+1];
        strncpy(this->sSol, exp.sSol, strlen(exp.sSol)+1);
    }
}

CubeSolutionException::~CubeSolutionException()
{
    if (sSol != NULL)
        delete[] sSol;
}

CubeSolutionException& CubeSolutionException::operator=(const CubeSolutionException& exp)
{
    if (this!=&exp) 
    {
        // clear old values if exist
        if (sSol != NULL)
            delete sSol;
        sSol = NULL;
            
        // execute operator= of parent class
        Exception::operator=(*((Exception*) &exp));
            
        // set new values
        if (exp.sSol != NULL) 
        {
            this->sSol = new char[strlen(exp.sSol)+1];
            strncpy(this->sSol, exp.sSol, strlen(exp.sSol)+1);
        }
        this->iValue = exp.iValue;
    }
    return *this;
}

Exception* CubeSolutionException::Clone()
{
    return new CubeSolutionException(*this);
}


////////////////////////////////////////////////////////////////////////////////
// TO_STRING - METHOD TO PRESENT CONTENT
////////////////////////////////////////////////////////////////////////////////

char* CubeSolutionException::ToString()
{
    const int STR_LEN = 300;
    int pos = 0;
    char *sExp = new char[STR_LEN];

    // Copy code error
    char* sExpBase = Exception::ToString();
    int txtpos = (strstr(sExpBase, "TXT") - sExpBase) / sizeof(char);
    strncpy(sExp, sExpBase, txtpos+1);
    pos += txtpos;
    
    // set cube state
    if (sSol)
    {
        strncpy(&sExp[pos], "SOL=[", 6);
        strncpy(&sExp[pos+5], sSol, strlen(sSol));
        pos += strlen(sSol);
        strncpy(&sExp[pos], "] ", 3);
        pos += 2;
    }

    // set value of error      
    strncpy(&sExp[pos], "VAL=+00 ", 9);
    sExp[pos+4] = ( iValue>=0 ? '+' : '-' );
    int value = ( iValue>0 ? iValue : -1*iValue );
    for (int i=0; value>=1; i++, value/=10)
    {
        if (i%4==3) i++;
        sExp[pos+6-i] = '0' + value%10;
    }
    pos += 8;

    // copy error text
    int txtlen = strlen(sExpBase) - txtpos;
    strncpy(&sExp[pos], &sExpBase[txtpos], ( txtlen > STR_LEN-pos ? STR_LEN-pos : txtlen+1 ));
    sExp[STR_LEN-1] = '\0';
    delete[] sExpBase;
   
    return sExp;
}


////////////////////////////////////////////////////////////////////////////////
// ERROR TEXTS AND ACCESS TO THEM
////////////////////////////////////////////////////////////////////////////////

const char* CubeSolutionException::ErrorText[] = 
  {  
	  "",  
	  "Invalid position of move in solution",
	  "Parsing failed on invalid move", 
	  "Parsing failed on invalid turn", 
	  "Parsing failed on invalid string format", 
  };

const char* CubeSolutionException::GetErrorText()
{
    if (iError > 0 && iError < ERR_CSOL_NUMBER)
        return ErrorText[iError];
    else 
        return "??";
}
