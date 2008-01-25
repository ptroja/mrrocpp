/////////////////////////////////////////////////////////
// Name: Permutation.cpp
// Implements : Permutation
/////////////////////////////////////////////////////////

#include "vsp/rcs/Combinatorials/Permutation.h"
#include "vsp/rcs/Combinatorials/CombinatorialsException.h"

#include <string>


const int Permutation::MAX_SIZE = 12;

////////////////////////////////////////////////////////////////////////////////
// BASE METHODS - CONSTRUCT, DESTRUCT
////////////////////////////////////////////////////////////////////////////////

Permutation::Permutation(int size)
    throw (CombinatorialsException)
    : Collection(size, size, MAX_SIZE, MAX_SIZE)
{}

Permutation::Permutation(const Permutation& perm) : Collection(perm)
{}


////////////////////////////////////////////////////////////////////////////////
// STATIC COUNT OF NUMBER OF POSSIBLE PERMUTATIONS
////////////////////////////////////////////////////////////////////////////////

int Permutation::NUMBER(int size) 
    throw (CombinatorialsException)
{
    int i, ret_val = 1;

    // Check parameter
    ValidateSize(size, MAX_SIZE, true);

    // Counts (size!)
    for (i=1; i<=size; i++)
        ret_val = ret_val * i;
    
    return ret_val;
}


////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION OF VIRTUAL METHODS FROM COLLECTION
////////////////////////////////////////////////////////////////////////////////

Collection* Permutation::Clone()     {  return new Permutation(*this);  }
bool Permutation::AllowComposition() {  return true;                    }
char Permutation::GetType()          {  return 'P';                     }

void Permutation::SetInitState()
{
    int i;
    
    // Set initial value of all elements
    for (i=0; i<iCollSize; i++)
        iaElems[i] = i;
}

void Permutation::FromOrdinal(int ord) 
     throw (CombinatorialsException)
{
    int i, coeffI;
    
    // Check parameters
    ValidateOrdinal(ord, NUMBER(iCollSize));

    // Set identity
    SetInitState();
        
    // Find relative position of next biggest element
    for (i = iCollSize-2; i >= 0; i--) 
    {
        // Get position of value i
        coeffI = ord % (iCollSize-i);
        // Delete position of value i from permution number
        ord /= (iCollSize-i);

        // Move bigger elements to give place to the current one
        if (coeffI > 0)
           memcpy(&iaElems[i], &iaElems[i+1], coeffI * sizeof(int));
        iaElems[i+coeffI] = i;
    }
}

int Permutation::ToOrdinal()
{
    int ord = 0;
    int i, coeffI = 0, limit, *elems = new int[iCollSize], temp;


    // Make a copy of the permutation vector
    memcpy(elems, iaElems, iCollSize * sizeof(int));

    // For next subpermutation without smallest values
    for (limit = 0; limit < iCollSize; limit++) 
    {
        // Find the minimum in current subpermutation
        temp = iCollSize;
        for (i = limit; i < iCollSize; i++) 
        {
            if (elems[i] < temp) 
            {
                temp = elems[i];
                coeffI = i;
            }
        }
        
        // Accumulate result
        ord = ord*(iCollSize-limit)+(coeffI-limit);

        // Move smallest element out of scope
        memmove(&elems[limit+1], &elems[limit], (coeffI-limit) * sizeof(int));
    }
    
    delete[] elems;
    return ord;
}

int Permutation::Parity() 
{
    int i,j;
    int parity = 0;
    
    for (i=0; i<iCollSize-1; i++) 
        for (j=i+1; j<iCollSize; j++) 
            if (iaElems[i] > iaElems[j]) 
                parity++;

    return parity % 2;
}

void Permutation::CheckTable(const int tab[]) 
     throw (CombinatorialsException)
{
    ValidatePermutation(tab, iCollSize);
}


////////////////////////////////////////////////////////////////////////////////
// PERMUTATION SPECIFIC CALCULATIONS - INVERSE, CYCLE
////////////////////////////////////////////////////////////////////////////////

void Permutation::Inverse()
{
    int i, *elems = new int[iCollSize];
     
    // Copy permutation
    memcpy(elems, iaElems, iCollSize*sizeof(int));
     
    // Set inverse permutation
    for (i=0; i<iCollSize; i++)
        iaElems[elems[i]] = i;
    
    delete[] elems;
}
             
void Permutation::FromCycle(const int cycle[], int C) 
     throw (CombinatorialsException)
{
     int i;
     
     // Check parameters
     ValidateCycle(cycle, C, iCollSize);
     
     // Set identity permutation
     for (i=0; i<iCollSize; i++)
         iaElems[i] = i;
     
     // Set cycle
     for (i=0; i<C; i++)
         iaElems[cycle[i]] = cycle[(i+1)%C];
}


////////////////////////////////////////////////////////////////////////////////
// STATIC VALIDATION METHODS
////////////////////////////////////////////////////////////////////////////////

void Permutation::ValidateCycle(const int cycle[], int C, int N)
     throw (CombinatorialsException)
{
    int i, *v = new int[N];
    
    // Validate only in validation mode
    if (!bValidation)
        return;
        
    // Check if C is allowed
    if (C <= 0) 
    {
        delete[] v;
        throw CombinatorialsException(NULL, C,
              CombinatorialsException::ERR_COMB_CYCLENOTPOS);
    }
    if (C > N) 
    {
        delete[] v;
        throw CombinatorialsException(NULL, C,
              CombinatorialsException::ERR_COMB_CYCLETOOBIG);
    }

    // Check if values in perm form a permutation
    for (i=0; i<N; i++)
        v[i] = 0;
    for (i=0; i<C; i++)
    {
        if (cycle[i] >= N)
        {
            delete[] v;
            throw CombinatorialsException(NULL, i, 
                  CombinatorialsException::ERR_COMB_CYCLEVALTOOBIG);
        }
        v[cycle[i]]++;
    }
    for (i=0; i<N; i++)
        if (v[i] > 1)
        {
            delete[] v;
            throw CombinatorialsException(NULL, i, 
                  CombinatorialsException::ERR_COMB_CYCLEVALREP);
        }
    
    delete[] v;
}

void Permutation::ValidatePermutation(const int perm[], int size)
     throw (CombinatorialsException)
{
    int i, *counter = new int[size];
    
    // Validate only in validation mode
    if (!bValidation)
    {
        delete[] counter;
        return;
    }
        
    // Check if values in perm form a permutation
    for (i=0; i<size; i++)
        counter[i] = 0;
    for (i=0; i<size; i++)
        counter[perm[i]]++;
    for (i=0; i<size; i++)
        if (counter[i] > 1)
        {
            delete[] counter;
            throw CombinatorialsException(NULL, i, 
                  CombinatorialsException::ERR_COMB_TABREP);
        }
    
    delete[] counter;
}
