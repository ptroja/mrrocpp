/////////////////////////////////////////////////////////
// Name: Variation.cpp
// Implements : Variation
/////////////////////////////////////////////////////////

#include "vsp/rcs/Combinatorials/Variation.h"
#include "vsp/rcs/Combinatorials/CombinatorialsException.h"

#include <string>


const int Variation::MAX_COLL_SIZE = 12;
const int Variation::MAX_SET_SIZE  =  6;

////////////////////////////////////////////////////////////////////////////////
// BASE METHODS - CONSTRUCT, DESTRUCT
////////////////////////////////////////////////////////////////////////////////

Variation::Variation(int collSize, int setSize)
    throw (CombinatorialsException)
    : Collection(collSize, setSize, MAX_COLL_SIZE, MAX_SET_SIZE)
{}

Variation::Variation(const Variation& vari) : Collection(vari)
{}


////////////////////////////////////////////////////////////////////////////////
// STATIC COUNT OF NUMBER OF POSSIBLE PERMUTATIONS
////////////////////////////////////////////////////////////////////////////////

int Variation::NUMBER(int collSize, int setSize) 
    throw (CombinatorialsException)
{
    int i, ret_val = 1;

    // Check parameters
    ValidateSize(collSize, MAX_COLL_SIZE);
    ValidateSize(setSize, MAX_SET_SIZE);

    // Counts (setSize^collSize)
    for (i=1; i<=collSize; i++)
        ret_val = ret_val * setSize;
    
    return ret_val;
}


////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION OF VIRTUAL METHODS FROM COLLECTION
////////////////////////////////////////////////////////////////////////////////

Collection* Variation::Clone()     {  return new Variation(*this);  }
bool Variation::AllowComposition() {  return false;                 }
char Variation::GetType()          {  return 'V';                   }

void Variation::SetInitState()
{
    int i;
    
    // Set initial value (0) of all elements
    for (i=0; i<iCollSize; i++)
        iaElems[i] = 0;
}

void Variation::FromOrdinal(int ord) 
     throw (CombinatorialsException)
{
    int i;
    
    // Check parameters
    ValidateOrdinal(ord, NUMBER(iCollSize, iSetSize));
    
    // Set values of elements in variation
    for (i=iCollSize-1; i>=0; i--)
    {
        iaElems[i] = ord%iSetSize;
        ord /= iSetSize;
    }
}

int Variation::ToOrdinal()
{
    int i, ord = 0;

    // Count ordinal
    for(i=0; i<iCollSize; i++)
        ord = ord * iSetSize + iaElems[i];
    
    return ord;
}

int Variation::Parity() 
{
    int i, par = 0;
    
    // Count ordinal
    for(i=0; i<iCollSize; i++)
        par += iaElems[i];

    return par % iSetSize;
}


////////////////////////////////////////////////////////////////////////////////
// VARIATION SPECIFIC CALCULATIONS - CHOOSENEXT
////////////////////////////////////////////////////////////////////////////////

void Variation::ChooseNext(int pos, bool bigger)
     throw (CombinatorialsException)
{
    // Check parameter
    ValidatePosition(pos, iCollSize);
    
    // Choose next value on position pos
    iaElems[pos] = (iaElems[pos] + (bigger ? 1 : -1) + iSetSize) % iSetSize;
}


////////////////////////////////////////////////////////////////////////////////
// STATIC VALIDATION METHODS
////////////////////////////////////////////////////////////////////////////////

void Variation::ValidatePosition(int pos, int size)
     throw (CombinatorialsException)
{
    // Validate only in validation mode
    if (!bValidation)
        return;
        
    // Check if pos is allowed
    if (pos < 0) 
       throw CombinatorialsException(NULL, pos,
             CombinatorialsException::ERR_COMB_POSNEG);
    if (pos >= size) 
       throw CombinatorialsException(NULL, pos,
             CombinatorialsException::ERR_COMB_POSTOOBIG);
}
