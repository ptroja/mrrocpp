/////////////////////////////////////////////////////////
// Nazwa: CombinatorialsTester.h
// Opis: Klasa CombinatorialsTester testow klas operacji
//       kombinatorycznych.
/////////////////////////////////////////////////////////

#ifndef CombinatorialsTester_h
#define CombinatorialsTester_h

#include "../General/Tester.h"


class Collection;
class CombinatorialsException;

// Klasa wyjatkow operacji kombinatorycznych.
class CombinatorialsTester : public Tester
{
      
  public:
         
    CombinatorialsTester() {}
    virtual ~CombinatorialsTester() {}
         
    virtual bool Test();
    
    
  private:
          
    bool TestPermutation();
    bool TestCombination();
    bool TestVariation();

    bool TestPermutationCreateException(int size);
    bool TestPermutationOrdinalException(int size, int ordinal);
    bool TestPermutationTableException(int size, const int tab[]);
    bool TestPermutationCreate(int size, int ordinal);
    bool TestPermutationOrdinal(int size, int ordinal);
    bool TestPermutationInverse(int size, int ordinal);
    bool TestPermutationCycle(int size, int ordinal, int csize, int cordinal);

    bool TestCombinationCreateException(int colSize, int setSize);
    bool TestCombinationOrdinalException(int colSize, int setSize, int ordinal);
    bool TestCombinationTableException(int colSize, int setSize, const int tab[]);
    bool TestCombinationCreate(int colSize, int setSize, int ordinal);
    bool TestCombinationOrdinal(int colSize, int setSize, int ordinal);

    bool TestVariationCreateException(int colSize, int setSize);
    bool TestVariationOrdinalException(int colSize, int setSize, int ordinal);
    bool TestVariationTableException(int colSize, int setSize, const int tab[]);
    bool TestVariationCreate(int colSize, int setSize, int ordinal);
    bool TestVariationOrdinal(int colSize, int setSize, int ordinal);

    void Print(const char *desc, Collection *coll);
    void Print(const char *desc, Collection *coll, int ordinal);
    void Print(const char *desc, CombinatorialsException *exp);
};


#endif // CombinatorialsTester_h
