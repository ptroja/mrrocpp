/////////////////////////////////////////////////////////
// Name: Collection.cpp
// Implements : Collection
/////////////////////////////////////////////////////////

#include "vsp/rcs/Combinatorials/Collection.h"
#include "vsp/rcs/Combinatorials/CombinatorialsException.h"

#include <string>


bool Collection::bValidation = true;

////////////////////////////////////////////////////////////////////////////////
// BASE METHODS - CONSTRUCT, DESTRUCT
////////////////////////////////////////////////////////////////////////////////

Collection::Collection(int collSize, int setSize, int maxCollSize, int maxSetSize)
    throw (CombinatorialsException)
    : iaElems(NULL), 
      iCollSize(collSize), 
      iSetSize(setSize)
{
    // Check parameters
    ValidateSize(collSize, maxCollSize);
    ValidateSize(setSize, maxSetSize);
    
    // Create table for collection elements
    iCollSize = collSize;
    iSetSize = setSize;
    iaElems = new int[iCollSize];

    // Set initial state of collection
    SetInitState();
}

Collection::Collection(const Collection& coll)
    : iaElems(NULL), 
      iCollSize(coll.iCollSize), 
      iSetSize(coll.iSetSize)
{
    // Copy table
    iaElems = new int[iCollSize];
    memcpy(iaElems, coll.iaElems, iCollSize*sizeof(int));
}

Collection::~Collection()
{
    // Free memory
    delete[] iaElems;
}

Collection& Collection::operator=(const Collection& coll)
{
    if (this!=&coll)
    {
        // Set sizes
        iCollSize = coll.iCollSize;
        iSetSize = coll.iSetSize;
    
        // Copy table
        if (iaElems != NULL) delete[] iaElems;
        iaElems = new int[iCollSize];
        memcpy(iaElems, coll.iaElems, iCollSize*sizeof(int));
    }
    return *this;
}

int Collection::operator==(const Collection& coll)
{
    // Compare table with collection's elements
    return (  (this->iCollSize == coll.iCollSize)
           && (this->iSetSize == coll.iSetSize)
           && !memcmp(this->iaElems, coll.iaElems, this->iCollSize * sizeof(int)) );
}

 
////////////////////////////////////////////////////////////////////////////////
// COLLECTION CALCULATIONS - INIT STATE, COMPOSITION AND TO/FROM TABLE
////////////////////////////////////////////////////////////////////////////////

void Collection::SetInitState()
{
    int i;
    
    // Clear all elements
    for (i=0; i<iCollSize; i++)
        iaElems[i] = 0;
}
 
void Collection::Composite(Collection& coll) 
     throw (CombinatorialsException)
{
    int i, *elems;
    
    elems = new int[iCollSize];
     
    // Check if coll allows composition
    if (!coll.AllowComposition())
       throw CombinatorialsException(&coll, -1, 
             CombinatorialsException::ERR_COMB_COMPFORBIDDEN);

    // Check if collections have the same size
    if (iCollSize != coll.iCollSize) 
       throw CombinatorialsException(&coll, iCollSize, 
             CombinatorialsException::ERR_COMB_COMPSIZEDIF);

    // Copy collection
    memcpy(elems, iaElems, iCollSize * sizeof(int));
     
    // Set composition of Collection and perm
    for (i=0; i<iCollSize; i++)
        iaElems[i] = elems[coll.iaElems[i]];
        
    delete[] elems;
}

void Collection::FromTable(const int tab[]) throw (CombinatorialsException)
{
    // Check table
    ValidateCollection(tab, iCollSize, iSetSize);
    CheckTable(tab);
    
    // Copy elements from table tab
    memcpy(iaElems, tab, iCollSize*sizeof(int));
}

void Collection::ToTable(int tab[])
{
    // Copy elements to table tab
    memcpy(tab, iaElems, iCollSize*sizeof(int));
}


////////////////////////////////////////////////////////////////////////////////
// STATIC VALIDATION METHODS
////////////////////////////////////////////////////////////////////////////////

void Collection::ValidateSize(int size, int max, bool zero)
     throw (CombinatorialsException)
{
    // Validate only in validation mode
    if (!bValidation)
        return;
        
    // Check if size is allowed
    if (size < 0) 
       throw CombinatorialsException(NULL, size, 
             CombinatorialsException::ERR_COMB_SIZENEG);
    if (!zero && size == 0) 
       throw CombinatorialsException(NULL, size, 
             CombinatorialsException::ERR_COMB_SIZEZERO);
    if (size > max) 
       throw CombinatorialsException(NULL, size, 
             CombinatorialsException::ERR_COMB_SIZETOOBIG);
}

void Collection::ValidateOrdinal(int ord, int max)
     throw (CombinatorialsException)
{
    // Validate only in validation mode
    if (!bValidation)
        return;
        
    // Check if ord is number of collection with size elements
    if (ord < 0)
       throw CombinatorialsException(NULL, ord, 
             CombinatorialsException::ERR_COMB_ORDNEG);
    if (ord >= max)
       throw CombinatorialsException(NULL, ord,
             CombinatorialsException::ERR_COMB_ORDTOOBIG);
}

void Collection::ValidateCollection(const int tab[], int collSize, int setSize) 
     throw (CombinatorialsException)
{
    int i;
    
    // Validate only in validation mode
    if (!bValidation)
        return;
        
    // Check if each od colSize elements from table tab is from set {0, setSize-1}
    for (i=0; i<collSize; i++)
    {
        if (tab[i] < 0)
            throw CombinatorialsException(NULL, i, 
                CombinatorialsException::ERR_COMB_TABNEG);
        if (tab[i] >= setSize)
            throw CombinatorialsException(NULL, i,
                CombinatorialsException::ERR_COMB_TABTOOBIG);
    }
}


////////////////////////////////////////////////////////////////////////////////
// TOSTRING METHOD FOR PRINTING COLLECTION
////////////////////////////////////////////////////////////////////////////////

char* Collection::ToString()
{
    const int HDR_LEN = 10;
    const char* STRING_FORMAT = "X(00,00)=[00,00,00,00,00,00,00,00,00,00,00,00]";

    int i, len = HDR_LEN + iCollSize*3 + 1; 
    char *sColl = new char[len];

    // Init string format
    strncpy(sColl, STRING_FORMAT, len);
    strncpy(&sColl[HDR_LEN + 3*iCollSize - 1], "]", 2);

    // Fill collection type
    sColl[0] = GetType();
    
    // Fill length of collection and of set on which collection is performed
    sColl[2] = iCollSize/10 + '0';
    sColl[3] = iCollSize%10 + '0';
    sColl[5] = iSetSize/10 + '0';
    sColl[6] = iSetSize%10 + '0';
    
    // Fill elements
    for (i=0; i<iCollSize; i++)
    {
        sColl[HDR_LEN + 3*i    ] = iaElems[i]/10 + '0';
        sColl[HDR_LEN + 3*i + 1] = iaElems[i]%10 + '0';
    }

    return sColl;
}
