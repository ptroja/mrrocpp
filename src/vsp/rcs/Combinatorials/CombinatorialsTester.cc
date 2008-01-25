/////////////////////////////////////////////////////////
// Name: CombinatorialsTester.cpp
// Implements : CombinatorialsTester
/////////////////////////////////////////////////////////

#include "vsp/rcs/Combinatorials/CombinatorialsTester.h"
#include "vsp/rcs/Combinatorials/Permutation.h"
#include "vsp/rcs/Combinatorials/Combination.h"
#include "vsp/rcs/Combinatorials/Variation.h"


#include <string.h>
#include <stdlib.h> 
#include <stdio.h>
#include <time.h>


////////////////////////////////////////////////////////////////////////////////
// MAIN TEST METHOD
////////////////////////////////////////////////////////////////////////////////

bool CombinatorialsTester::Test()
{
    printf("TEST COMBINATORIALS\n\n");
    bool result = true;

    // initialize random seed
    srand (time(NULL));
    
    if (!TestPermutation()) result = false;
    if (!TestCombination()) result = false;
    if (!TestVariation())   result = false;

    printf("TEST COMBINATORIALS : %s\n\n\n", ResultText(result));
    if (bLog) printf("\n\n");
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE PERMUTATION TESTS
////////////////////////////////////////////////////////////////////////////////

bool CombinatorialsTester::TestPermutation()
{
    printf("TEST PERMUTATION\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;
    
    const int MAX_SIZE = Permutation::MAX_SIZE;
    const int TEST_SIZE = 8;
    const int TEST_MAXORD = Permutation::NUMBER(TEST_SIZE);
    const int CYCLE_SIZE = 4;
    const int CYCLE_MAXORD = Permutation::NUMBER(CYCLE_SIZE);


    // Test creation exceptions
    if (bLog) printf("TEST CreateException\n");
    subres = TestPermutationCreateException(0);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestPermutationCreateException(-1);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestPermutationCreateException(MAX_SIZE+1);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test ordinal set exceptions
    if (bLog) printf("TEST OrdinalException\n");
    subres = TestPermutationOrdinalException(MAX_SIZE, -1);
    printf("TEST OrdinalException - %s\n", ResultText(subres));
    if (!subres) result = false;

    subres = TestPermutationOrdinalException(MAX_SIZE, Permutation::NUMBER(MAX_SIZE));
    printf("TEST OrdinalException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test table set exception
    if (bLog) printf("TEST TableException\n");
    subres = true;
    const int BAD_IPERM[3][12] = {
        { 0,1,2,3,4,5,6,7,8,9,10,12 },
        { 0,1,2,3,4,5,6,7,8,9,10,-1 },
        { 0,1,2,3,4,5,6,7,8,9,10,10 },
    };
    for (int i=0; i<3; i++)
    {
        subres2 = TestPermutationTableException(MAX_SIZE, BAD_IPERM[i]);
        printf("TEST TableException (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST TableException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test permutation creation
    if (bLog) printf("TEST Create\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestPermutationCreate(TEST_SIZE, rand()%TEST_MAXORD);
        printf("TEST Create (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Create - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test permutation ordinals
    if (bLog) printf("TEST Ordinal&Table\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestPermutationOrdinal(TEST_SIZE, rand()%TEST_MAXORD);
        printf("TEST Ordinal&Table (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Ordinal&Table - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test permutation inverse and composition
    if (bLog) printf("TEST Inverse&Composition\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestPermutationInverse(TEST_SIZE, rand()%TEST_MAXORD);
        printf("TEST Inverse&Composition (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Inverse&Composition - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test permutation cycle
    if (bLog) printf("TEST Cycle\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestPermutationCycle(TEST_SIZE, rand()%TEST_MAXORD, CYCLE_SIZE, rand()%CYCLE_MAXORD);
        printf("TEST Cycle (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Cycle - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST PERMUTATION : %s\n\n", ResultText(result));
    if (bLog) printf("\n");
    return result;
}

bool CombinatorialsTester::TestPermutationCreateException(int size)
{
    if (bLog) printf("\tparams: %d\n", size);
    try 
    {
        Permutation perm(size);
    } 
    catch (CombinatorialsException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CombinatorialsTester::TestPermutationOrdinalException(int size, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d\n", size, ordinal);
    try 
    {
        Permutation perm(size);
        perm.FromOrdinal(ordinal);
    } 
    catch (CombinatorialsException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CombinatorialsTester::TestPermutationTableException(int size, const int tab[])
{
    if (bLog)
    {
        printf("\tparams: %d, [", size);
        for (int i=0; i<size; i++)
             printf("%d%c", tab[i], ( i==size-1 ? '.' : ',' ));
        printf("]\n");
    }
    try 
    {
        Permutation perm(size);
        perm.FromTable(tab);
    } 
    catch (CombinatorialsException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CombinatorialsTester::TestPermutationCreate(int size, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d\n", size, ordinal);
    bool result = true;
    
    try 
    {
        Permutation perm(size);
        perm.FromOrdinal(ordinal);
        Print("P1", &perm);
        Permutation perm2(perm);
        Print("P2", &perm2);
        if (!(perm==perm2)) result = false;
        Permutation *perm3 = (Permutation*) perm.Clone();
        Print("P3", perm3);
        if (!(perm==*perm3)) result = false;
        delete perm3;
    } 
    catch (CombinatorialsException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}

bool CombinatorialsTester::TestPermutationOrdinal(int size, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d\n", size, ordinal);
    bool result = true;
    
    try 
    {
        // permutation from ordinal to table
        Permutation perm1(size), perm2(size);
        perm1.FromOrdinal(ordinal);
        Print("P1", &perm1, ordinal);
        int *ptable = new int[size];
        perm1.ToTable(ptable);
        
        // permutation from table to ordinal
        perm2.FromTable(ptable);
        int ord2 = perm2.ToOrdinal();
        Print("P2", &perm2, ord2);
        result = (ordinal == ord2 && perm1 == perm2);
        
        delete[] ptable;
    } 
    catch (CombinatorialsException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}

bool CombinatorialsTester::TestPermutationInverse(int size, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d\n", size, ordinal);
    bool result = true;
    
    try 
    {
        // permutation from ord
        Permutation perm1(size), perm2(size);
        perm1.FromOrdinal(ordinal);
        Print("PERM", &perm1, ordinal);
        
        // reverse permutation to ord
        perm2.FromOrdinal(ordinal);
        perm2.Inverse();
        int ord2 = perm2.ToOrdinal();
        Print("REVS", &perm2, ord2);
        
        // composition
        perm1.Composite(perm2);
        ord2 = perm1.ToOrdinal();
        Print("COMB", &perm1, ord2);
        
        result = (ord2 == 0);
    } 
    catch (CombinatorialsException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}

bool CombinatorialsTester::TestPermutationCycle(int size, int ordinal, int csize, int cordinal)
{
    if (bLog) printf("\tparams: %d,%d,%d,%d\n", size, ordinal, csize, cordinal);
    bool result = true;
    
    try 
    {
        // create cycle
        Permutation cperm(csize);
        cperm.FromOrdinal(cordinal);
        int *ctable = new int[csize];
        cperm.ToTable(ctable);
        Permutation cycle(size);
        cycle.FromCycle(ctable, csize);
        delete[] ctable;
        Print("CYCL", &cycle);
        
        // permutation from ord
        Permutation perm(size);
        perm.FromOrdinal(ordinal);
        Print("PERM", &perm);
        
        // composition until identity
        for (int i=0; i<csize; i++)
        {
            perm.Composite(cycle);
            Print("PCYC", &perm);
        }
        
        result = (perm.ToOrdinal() == ordinal);
    } 
    catch (CombinatorialsException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE COMBINATION TESTS
////////////////////////////////////////////////////////////////////////////////

bool CombinatorialsTester::TestCombination()
{
    printf("TEST COMBINATION\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;
    
    const int MAX_COLSIZE = Combination::MAX_COLL_SIZE;
    const int MAX_SETSIZE = Combination::MAX_SET_SIZE;
    const int TEST_COLSIZE = 6;
    const int TEST_SETSIZE = 12;
    const int TEST_MAXORD = Combination::NUMBER(TEST_COLSIZE, TEST_SETSIZE);


    // Test creation exceptions
    if (bLog) printf("TEST CreateException\n");
    subres = TestCombinationCreateException(0, MAX_SETSIZE);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestCombinationCreateException(-1, MAX_SETSIZE);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestCombinationCreateException(MAX_COLSIZE+1, MAX_SETSIZE);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;

    subres = TestCombinationCreateException(MAX_COLSIZE, 0);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestCombinationCreateException(MAX_COLSIZE, -1);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestCombinationCreateException(MAX_COLSIZE, MAX_SETSIZE+1);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestCombinationCreateException(MAX_COLSIZE, MAX_COLSIZE-1);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test ordinal set exceptions
    if (bLog) printf("TEST OrdinalException\n");
    subres = TestCombinationOrdinalException(MAX_COLSIZE, MAX_SETSIZE, -1);
    printf("TEST OrdinalException - %s\n", ResultText(subres));
    if (!subres) result = false;

    subres = TestCombinationOrdinalException(MAX_COLSIZE, MAX_SETSIZE, Combination::NUMBER(MAX_COLSIZE, MAX_SETSIZE));
    printf("TEST OrdinalException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test table set exception
    if (bLog) printf("TEST TableException\n");
    subres = true;
    const int BAD_ICOMB[3][8] = {
        { 0,1,2,3,4,5,6,17 },
        { 0,1,2,3,4,5,6,-1 },
        { 0,1,2,3,4,5,6,6 },
    };
    for (int i=0; i<3; i++)
    {
        subres2 = TestCombinationTableException(MAX_COLSIZE, MAX_SETSIZE, BAD_ICOMB[i]);
        printf("TEST TableException (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST TableException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test combination creation
    if (bLog) printf("TEST Create\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestCombinationCreate(TEST_COLSIZE, TEST_SETSIZE, rand()%TEST_MAXORD);
        printf("TEST Create (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Create - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test combination ordinals
    if (bLog) printf("TEST Ordinal&Table\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestCombinationOrdinal(TEST_COLSIZE, TEST_SETSIZE, rand()%TEST_MAXORD);
        printf("TEST Ordinal&Table (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Ordinal&Table - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST COMBINATION : %s\n\n", ResultText(subres));
    if (bLog) printf("\n");
    return result;
}

bool CombinatorialsTester::TestCombinationCreateException(int collSize, int setSize)
{
    if (bLog) printf("\tparams: %d,%d\n", collSize, setSize);
    try 
    {
        Combination comb(collSize, setSize);
    } 
    catch (CombinatorialsException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CombinatorialsTester::TestCombinationOrdinalException(int collSize, int setSize, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d,%d\n", collSize, setSize, ordinal);
    try 
    {
        Combination comb(collSize, setSize);
        comb.FromOrdinal(ordinal);
    } 
    catch (CombinatorialsException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CombinatorialsTester::TestCombinationTableException(int collSize, int setSize, const int tab[])
{
    if (bLog)
    {
        printf("\tparams: %d,%d,[", collSize, setSize);
        for (int i=0; i<collSize; i++)
             printf("%d%c", tab[i], ( i==collSize-1 ? '.' : ',' ));
        printf("]\n");
    }
    try 
    {
        Combination comb(collSize, setSize);
        comb.FromTable(tab);
    } 
    catch (CombinatorialsException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CombinatorialsTester::TestCombinationCreate(int collSize, int setSize, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d,%d\n", collSize, setSize, ordinal);
    bool result = true;
    
    try 
    {
        // Create comb, set value from ordinal
        Combination comb(collSize, setSize);
        comb.FromOrdinal(ordinal);
        Print("C1", &comb);
        
        // Create comb2 from comb
        Combination comb2(comb);
        Print("C2", &comb2);
        if (!(comb==comb2)) result = false;
        
        // Clone comb3 from comb
        Combination *comb3 = (Combination*) comb.Clone();
        Print("C3", comb3);
        if (!(comb==*comb3)) result = false;
        delete comb3;
    } 
    catch (CombinatorialsException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}

bool CombinatorialsTester::TestCombinationOrdinal(int collSize, int setSize, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d,%d\n", collSize, setSize, ordinal);
    bool result = true;
    
    try 
    {
        // combination from ordinal to table
        Combination comb1(collSize, setSize), comb2(collSize, setSize);
        comb1.FromOrdinal(ordinal);
        Print("C1", &comb1, ordinal);
        int *ctable = new int[collSize];
        comb1.ToTable(ctable);
        
        // combination from table to ordinal
        comb2.FromTable(ctable);
        int ord2 = comb2.ToOrdinal();
        Print("C2", &comb2, ord2);
        result = (ordinal == ord2 && comb1 == comb2);
        
        delete[] ctable;
    } 
    catch (CombinatorialsException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE VARIATION TESTS
////////////////////////////////////////////////////////////////////////////////

bool CombinatorialsTester::TestVariation()
{
    printf("TEST VARIATION\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;
    
    const int MAX_COLSIZE = Variation::MAX_COLL_SIZE;
    const int MAX_SETSIZE = Variation::MAX_SET_SIZE;
    const int TEST_COLSIZE = 8;
    const int TEST_SETSIZE = 3;
    const int TEST_MAXORD = Variation::NUMBER(TEST_COLSIZE, TEST_SETSIZE);


    // Test creation exceptions
    if (bLog) printf("TEST CreateException\n");
    subres = TestVariationCreateException(0, MAX_SETSIZE);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestVariationCreateException(-1, MAX_SETSIZE);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestVariationCreateException(MAX_COLSIZE+1, MAX_SETSIZE);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;

    subres = TestVariationCreateException(MAX_COLSIZE, 0);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestVariationCreateException(MAX_COLSIZE, -1);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    
    subres = TestVariationCreateException(MAX_COLSIZE, MAX_SETSIZE+1);
    printf("TEST CreateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test ordinal set exceptions
    if (bLog) printf("TEST OrdinalException\n");
    subres = TestVariationOrdinalException(MAX_COLSIZE, MAX_SETSIZE, -1);
    printf("TEST OrdinalException - %s\n", ResultText(subres));
    if (!subres) result = false;

    subres = TestVariationOrdinalException(MAX_COLSIZE, MAX_SETSIZE, Variation::NUMBER(MAX_COLSIZE, MAX_SETSIZE));
    printf("TEST OrdinalException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test table set exception
    if (bLog) printf("TEST TableException\n");
    subres = true;
    const int BAD_IVARI[2][12] = {
        { 0,0,0,0,0,0,0,0,0,0,0,6 },
        { 0,0,0,0,0,0,0,0,0,0,0,-1 },
    };
    for (int i=0; i<2; i++)
    {
        subres2 = TestVariationTableException(MAX_COLSIZE, MAX_SETSIZE, BAD_IVARI[i]);
        printf("TEST TableException (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST TableException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test variation creation
    if (bLog) printf("TEST Create\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestVariationCreate(TEST_COLSIZE, TEST_SETSIZE, rand()%TEST_MAXORD);
        printf("TEST Create (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Create - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test variation ordinals
    if (bLog) printf("TEST Ordinal&Table\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestVariationOrdinal(TEST_COLSIZE, TEST_SETSIZE, rand()%TEST_MAXORD);
        printf("TEST Ordinal&Table (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Ordinal&Table - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST VARIATION : %s\n\n", ResultText(subres));
    if (bLog) printf("\n");
    return result;
}

bool CombinatorialsTester::TestVariationCreateException(int collSize, int setSize)
{
    if (bLog) printf("\tparams: %d,%d\n", collSize, setSize);
    try 
    {
        Variation vari(collSize, setSize);
    } 
    catch (CombinatorialsException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CombinatorialsTester::TestVariationOrdinalException(int collSize, int setSize, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d,%d\n", collSize, setSize, ordinal);
    try 
    {
        Variation vari(collSize, setSize);
        vari.FromOrdinal(ordinal);
    } 
    catch (CombinatorialsException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CombinatorialsTester::TestVariationTableException(int collSize, int setSize, const int tab[])
{
    if (bLog)
    {
        printf("\tparams: %d,%d.[", collSize, setSize);
        for (int i=0; i<collSize; i++)
             printf("%d%c", tab[i], ( i==collSize-1 ? '.' : ',' ));
        printf("]\n");
    }
    try 
    {
        Variation vari(collSize, setSize);
        vari.FromTable(tab);
    } 
    catch (CombinatorialsException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CombinatorialsTester::TestVariationCreate(int collSize, int setSize, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d,%d\n", collSize, setSize, ordinal);
    bool result = true;
    
    try 
    {
        // Create comb, set value from ordinal
        Variation vari(collSize, setSize);
        vari.FromOrdinal(ordinal);
        Print("C1", &vari);
        
        // Create comb2 from comb
        Variation vari2(vari);
        Print("C2", &vari2);
        if (!(vari==vari2)) result = false;
        
        // Clone comb3 from comb
        Variation *vari3 = (Variation*) vari.Clone();
        Print("C3", vari3);
        if (!(vari==*vari3)) result = false;
        delete vari3;
    } 
    catch (CombinatorialsException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}

bool CombinatorialsTester::TestVariationOrdinal(int collSize, int setSize, int ordinal)
{
    if (bLog) printf("\tparams: %d,%d,%d\n", collSize, setSize, ordinal);
    bool result = true;
    
    try 
    {
        // combination from ordinal to table
        Variation vari1(collSize, setSize), vari2(collSize, setSize);
        vari1.FromOrdinal(ordinal);
        Print("C1", &vari1, ordinal);
        int *vtable = new int[collSize];
        vari1.ToTable(vtable);
        
        // combination from table to ordinal
        vari2.FromTable(vtable);
        int ord2 = vari2.ToOrdinal();
        Print("C2", &vari2, ord2);
        result = (ordinal == ord2 && vari1 == vari2);
        
        delete[] vtable;
    } 
    catch (CombinatorialsException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}




////////////////////////////////////////////////////////////////////////////////
// PRINT OBJECT
////////////////////////////////////////////////////////////////////////////////

void CombinatorialsTester::Print(const char *desc, Collection *coll)
{
    if (!bLog) return;
    char *str = coll->ToString();
    printf("\t%s COLL: %s\n", (desc!=NULL ? desc : ""), str);
    delete[] str;
}

void CombinatorialsTester::Print(const char *desc, Collection *coll, int ordinal)
{
    if (!bLog) return;
    char *str = coll->ToString();
    printf("\t%s COLL[%d]: %s\n", (desc!=NULL ? desc : ""), ordinal, str);
    delete[] str;
}

void CombinatorialsTester::Print(const char *desc, CombinatorialsException *exp)
{
    if (!bLog) return;
    char *str = exp->ToString();
    printf("\t%s EXC: %s\n", (desc!=NULL ? desc : ""), str);
    delete[] str;
}


