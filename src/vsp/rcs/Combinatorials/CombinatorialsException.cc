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
    "success",
    "size cannot be negative",
    "size cannot be eqal to zero",
    "size is bigger than supported",
    "composition is forbidden with this collection",
    "composition only allowed for the same size collections",
    "ordinal number cannot be negative", 
    "ordinal number cannot be bigger that number of collections",
    "cycle size must be positive",
    "cycle cannot be longer than permutation",
    "values in cycle must be from permutation's set",
    "values in cycle cannot be repeated",
    "collection size must be smaller than set size for combination",
    "position number cannot be negative",
    "position number must be smaller than size of variation",
    "table with collection contains too small elements",
    "table with collection contains too big elements",
    "table with permutation/combination contains repeated values"
  };

const char* CombinatorialsException::GetErrorText()
{
    if (iError >= 0 && iError < ERR_COMB_NUMBER)
        return ErrorText[iError];
    else 
        return "??";
}
