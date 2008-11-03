/////////////////////////////////////////////////////////
// Name: CombinatorialsException.cpp
// Implements : CombinatorialsException
/////////////////////////////////////////////////////////

#include "vsp/rcs/Combinatorials/CombinatorialsException.h"
#include "vsp/rcs/Combinatorials/Collection.h"

#include <string.h>


const char *CombinatorialsException::TYPE = "Combinatorials";

////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, COPY and private helpers for them
////////////////////////////////////////////////////////////////////////////////

CombinatorialsException::CombinatorialsException(Collection* coll, int value, int error)
    : Exception(error), pColl(coll), iValue(value)
{}

CombinatorialsException::CombinatorialsException(const CombinatorialsException& exp)
    : Exception(*((Exception*) &exp)), pColl(exp.pColl), iValue(exp.iValue)
{}

CombinatorialsException::~CombinatorialsException()
{
    if (pColl) delete(pColl);
}

CombinatorialsException& CombinatorialsException::operator=(const CombinatorialsException& exp)
{
    if (this!=&exp)
    {
        if (pColl) delete(pColl);
        Exception::operator=(*((Exception*) &exp));
        if (exp.pColl != NULL)
            pColl = exp.pColl->Clone();
        else
            pColl = NULL;
        iValue = exp.iValue;
    }
    return *this;
}

Exception* CombinatorialsException::Clone()
{
    return new CombinatorialsException(*this);
}


////////////////////////////////////////////////////////////////////////////////
// TO_STRING - METHOD TO PRESENT CONTENT
////////////////////////////////////////////////////////////////////////////////

char* CombinatorialsException::ToString()
{
    const int STR_LEN = 200;
    char *sExp = new char[STR_LEN];
    int pos = 0;

    // Copy code error
    char* sExpBase = Exception::ToString();
    int txtpos = (strstr(sExpBase, "TXT") - sExpBase) / sizeof(char);
    strncpy(sExp, sExpBase, txtpos+1);
    pos += txtpos;
    
    // Fill collection
    if (pColl)
    {
        char *sColl = pColl->ToString();
        int collen = strlen(sColl);
        strncpy(&sExp[pos], sColl, collen);
        delete[] sColl;
        pos += collen;
    }
  
    // Set value of error      
    strncpy(&sExp[pos], "VAL=+0 000 000 000 ", 20);
    sExp[pos+4] = ( iValue>=0 ? '+' : '-' );
    int value = ( iValue>0 ? iValue : -1*iValue );
    for (int i=0; value>=1; i++, value/=10)
    {
        if (i%4==3) i++;
        sExp[pos+17-i] = '0' + value%10;
    }
    pos += 19;

    // Copy error text
    int txtlen = strlen(sExpBase) - txtpos;
    strncpy(&sExp[pos], &sExpBase[txtpos], ( txtlen > STR_LEN-pos ? STR_LEN-pos : txtlen+1 ));
    sExp[STR_LEN-1] = '\0';
    delete[] sExpBase;
   
    return sExp;
}


////////////////////////////////////////////////////////////////////////////////
// ERROR TEXTS AND ACCESS TO THEM
////////////////////////////////////////////////////////////////////////////////

char *CombinatorialsException::ErrorText[] = 
  { 
	(char*) "success",
	(char*) "size cannot be negative",
	(char*) "size cannot be eqal to zero",
	(char*) "size is bigger than supported",
	(char*) "composition is forbidden with this collection",
	(char*) "composition only allowed for the same size collections",
	(char*) "ordinal number cannot be negative", 
	(char*) "ordinal number cannot be bigger that number of collections",
	(char*) "cycle size must be positive",
	(char*) "cycle cannot be longer than permutation",
	(char*) "values in cycle must be from permutation's set",
	(char*) "values in cycle cannot be repeated",
	(char*) "collection size must be smaller than set size for combination",
	(char*) "position number cannot be negative",
	(char*) "position number must be smaller than size of variation",
	(char*) "table with collection contains too small elements",
	(char*) "table with collection contains too big elements",
	(char*) "table with permutation/combination contains repeated values"
  };

const char* CombinatorialsException::GetErrorText()
{
    if (iError >= 0 && iError < ERR_COMB_NUMBER)
        return ErrorText[iError];
    else 
        return "??";
}
