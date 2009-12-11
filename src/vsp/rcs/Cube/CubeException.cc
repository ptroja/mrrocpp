/////////////////////////////////////////////////////////
// Name: CubeException.h
// Implements: CubeException
/////////////////////////////////////////////////////////

#include "vsp/rcs/Cube/CubeException.h"
#include "vsp/rcs/Cube/Cube.h"

#include <string.h>


const char *CubeException::TYPE = "Cube";

////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, COPY and private helpers for them
////////////////////////////////////////////////////////////////////////////////

CubeException::CubeException(Cube *pCube, const char *sCube, int val, int error)
    : Exception(error), pException(NULL), pCube(NULL), sCube(NULL), iValue(val)
{
    if (pCube != NULL)
        this->pCube = pCube->Clone();
    
    if (sCube != NULL) 
    {
        this->sCube = new char[strlen(sCube)+1];
        strncpy(this->sCube, sCube, strlen(sCube)+1);
    }

    if (pException != NULL)
        this->pException = pException->Clone();
}

CubeException::CubeException(Exception *pExc, int error)
    : Exception(error), pException(NULL), pCube(NULL), sCube(NULL), iValue(0)
{
    pException = pExc->Clone();
}

CubeException::CubeException(const CubeException& exp)
    : Exception(exp.iError), pException(NULL), pCube(NULL), sCube(NULL), iValue(exp.iValue)
{
    if (exp.pCube != NULL)
        this->pCube = exp.pCube->Clone();
    
    if (exp.sCube != NULL) 
    {
        this->sCube = new char[strlen(exp.sCube)+1];
        strncpy(this->sCube, exp.sCube, strlen(exp.sCube)+1);
    }

    if (exp.pException != NULL)
        this->pException = exp.pException->Clone();
}

CubeException::~CubeException()
{
    if (pCube != NULL)
        delete pCube;
    if (sCube != NULL)
        delete[] sCube;
    if (pException != NULL)
        delete pException;
}

CubeException& CubeException::operator=(const CubeException& exp)
{
    if (this!=&exp) 
    {
        // clear old values if exist
        if (pCube != NULL)
            delete pCube;
        if (sCube != NULL)
            delete sCube;
        if (pException != NULL)
            delete pException;
            
        // execute operator= of parent class
        Exception::operator=(*((Exception*) &exp));
            
        // set new values
        if (exp.pCube != NULL)
            this->pCube = exp.pCube->Clone();
        
        if (exp.sCube != NULL) 
        {
            this->sCube = new char[strlen(exp.sCube)+1];
            strncpy(this->sCube, exp.sCube, strlen(exp.sCube)+1);
        }
    
        if (exp.pException != NULL)
            this->pException = exp.pException->Clone();
        
        this->iValue = exp.iValue;
    }
    return *this;
}

Exception* CubeException::Clone()
{
    return new CubeException(*this);
}


////////////////////////////////////////////////////////////////////////////////
// TO_STRING - METHOD TO PRESENT CONTENT
////////////////////////////////////////////////////////////////////////////////

char* CubeException::ToString()
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
    if (pCube || sCube)
    {
        strncpy(&sExp[pos], "CUBE=T[", 8);
        if (pCube)
        {
            sExp[pos+5] = 'P';
            char* sCube = pCube->ToString();
            strncpy(&sExp[pos+7], sCube, strlen(sCube));
            pos += strlen(sCube);
            delete[] sCube; 
        }
        else //if (sCube)
        {
            sExp[pos+5] = 'S';
            strncpy(&sExp[pos+7], sCube, strlen(sCube));
        }
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
   
    // set cube state
    if (pException)
    {
        char* sSrcExp = pException->ToString();
        char *sExpTmp = sExp;
        sExp = new char[strlen(sExpTmp)+strlen(sSrcExp)+13];
        pos = 0;
        strncpy(sExp, sExpTmp, strlen(sExpTmp));
        pos += strlen(sExpTmp);
        strncpy(&sExp[pos], " CAUSED BY: ", 12);
        pos += 12;
        strncpy(&sExp[pos], sSrcExp, strlen(sSrcExp));
        pos += strlen(sSrcExp);
        sExp[pos] = '\0';
        delete[] sExpTmp;
        delete[] sSrcExp;
    }
      
    return sExp;
}


////////////////////////////////////////////////////////////////////////////////
// ERROR TEXTS AND ACCESS TO THEM
////////////////////////////////////////////////////////////////////////////////

const char* CubeException::ErrorText[] = 
  {  
	  "",  
	  "Facelet marking does not match any center marking", 
	  "There must be 9 facelets for each marking", 
	  "Duplicate center marking", 
	  "Invalid corner markings", 
	  "Invalid corner orientation parity", 
	  "Invalid edge markings", 
	  "Invalid edge orientation parity", 
	  "Invalid total permutation parity", 
	  "Invalid face", 
	  "Invalid facelet", 
	  "Invalid face name", 
	  "Parsing failed on NULL string", 
	  "Parsing failed on too short string", 
	  "Parsing failed on invalid face", 
	  "Parsing failed on invalid facelet", 
	  "Parsing failed on invalid string format", 
	  "Parsing failed on repeated face name",
	  "Table contains elements with too small values",
	  "Table contains elements with too big values",
	  "Move is not implemented on FaceletCube",
	  "Table does not contain valid cube state",
	  "Invalid corner parity",
	  "Invalid edge parity",
	  "Unequal parity of corner and edge permutation",
	  "Invalid move name",
	  "Invalid turn name"
  };

const char* CubeException::GetErrorText()
{
    if (iError > 0 && iError < ERR_CUBE_NUMBER)
        return ErrorText[iError];
    else 
        return "??";
}
