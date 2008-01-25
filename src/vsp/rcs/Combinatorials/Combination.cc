/////////////////////////////////////////////////////////
// Name: Combination.cpp
// Implements : Combination
/////////////////////////////////////////////////////////

#include "vsp/rcs/Combinatorials/Combination.h"
#include "vsp/rcs/Combinatorials/CombinatorialsException.h"

#include <string>


const int Combination::MAX_COLL_SIZE =  8;
const int Combination::MAX_SET_SIZE  = 16;

////////////////////////////////////////////////////////////////////////////////
// BASE METHODS - CONSTRUCT, DESTRUCT
////////////////////////////////////////////////////////////////////////////////

Combination::Combination(int collSize, int setSize)
    throw (CombinatorialsException)
    : Collection(collSize, setSize, MAX_COLL_SIZE, MAX_SET_SIZE)
{ 
    // Check parameters
    if (collSize > setSize) 
       throw CombinatorialsException(NULL, collSize, 
             CombinatorialsException::ERR_COMB_NOTSUBSET);
}

Combination::Combination(const Combination& comb) : Collection(comb)
{}


////////////////////////////////////////////////////////////////////////////////
// STATIC COUNT OF NUMBER OF POSSIBLE PERMUTATIONS
////////////////////////////////////////////////////////////////////////////////

int Combination::NUMBER(int collSize, int setSize) 
    throw (CombinatorialsException)
{
    int ret_val = 1;
    int N = setSize, K = collSize;
    int NoverKfact = N; // Iterates from N down to K+1 to compute N! / (N-K)!
    int Kfact = 1;      // Iterates from 1 to K to divide out the K! term

    // Check parameters
    ValidateSize(collSize, MAX_COLL_SIZE, true);
    ValidateSize(setSize, MAX_SET_SIZE, true);
    if (collSize > setSize) 
       throw CombinatorialsException(NULL, collSize, 
             CombinatorialsException::ERR_COMB_NOTSUBSET);
    
    // Optimization
    if (K > N/2) 
        K = N-K;   

    // Calculations
    while (NoverKfact > K)
    {
        ret_val *= NoverKfact--; // Work on the N! / (N-K)! part
        ret_val /= Kfact++;      // Divide out the K! part
    }
    
    return ret_val;
}


////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION OF VIRTUAL METHODS FROM COLLECTION
////////////////////////////////////////////////////////////////////////////////

Collection* Combination::Clone()     {  return new Combination(*this);  }
bool Combination::AllowComposition() {  return false;                   }
char Combination::GetType()          {  return 'C';                     }

void Combination::SetInitState()
{
    int i;
    
    // Set initial value of all elements - choose first iCollSize elements
    for (i=0; i<iCollSize; i++)
        iaElems[i] = i;
}

void Combination::FromOrdinal(int ord) 
     throw (CombinatorialsException)
{
    int i,j, num;
    
    // Check parameters
    ValidateOrdinal(ord, NUMBER(iCollSize, iSetSize));

    // Set invalid elements in combination
    for (i=0; i<iCollSize; i++)
        iaElems[i] = iSetSize;

    for (i=0,j=0; i<iSetSize && j<iCollSize; i++)
    {
        num = NUMBER(iCollSize-j-1, iSetSize-i-1);
        if (ord >= num)
            ord -= num;
        else
            iaElems[j++] = i;
    }
}

int Combination::ToOrdinal()
{
    int i,j, *chosen = new int[iSetSize], ord = 0;

    // Mark chosen elements
    for (i=0; i<iSetSize; i++)
        chosen[i] = 0;
    for (i=0; i<iCollSize; i++)
        chosen[iaElems[i]]++;

    // Calculate combination number
    for (i=0, j=0; i<iSetSize && j<iCollSize; i++)
    {
        if (chosen[i])
            j++;
        else
            ord += NUMBER(iCollSize-j-1, iSetSize-i-1);
    }
    
    delete[] chosen;
    return ord;
}

int Combination::Parity() 
{
    // ??
    return 0;
}

void Combination::CheckTable(const int tab[]) 
     throw (CombinatorialsException)
{
    ValidateCombination(tab, iCollSize, iSetSize);
}


////////////////////////////////////////////////////////////////////////////////
// STATIC VALIDATION METHODS
////////////////////////////////////////////////////////////////////////////////

void Combination::ValidateCombination(const int comb[], int collSize, int setSize)
     throw (CombinatorialsException)
{
    int i, *counter = new int[setSize];
    
    // Validate only in validation mode
    if (!bValidation)
    {
        delete[] counter;
        return;
    }
        
    // Check if values in perm form a permutation
    for (i=0; i<setSize; i++)
        counter[i] = 0;
    for (i=0; i<collSize; i++)
        counter[comb[i]]++;
    for (i=0; i<setSize; i++)
        if (counter[i] > 1)
        {
            delete[] counter;
            throw CombinatorialsException(NULL, i, 
                  CombinatorialsException::ERR_COMB_TABREP);
        }
    
    delete[] counter;
}
