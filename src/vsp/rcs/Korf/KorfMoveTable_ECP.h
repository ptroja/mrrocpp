/////////////////////////////////////////////////////////
// Nazwa: KorfMoveTable_ECP.h
// Opis: Klasa KorfMoveTable_ECP reprezentujaca tablice
//       przeksztalcen kombinacji i permutacji polowy
//       krawedzi kostki Korfa. 
/////////////////////////////////////////////////////////

#ifndef KorfMoveTable_ECP_h
#define KorfMoveTable_ECP_h

#include "KorfMoveTable.h"
#include "../Combinatorials/Combination.h"


// Klasa tablicy przeksztalcen wspolrzednych kombinacji polowy krawedzi
// przez pojedyncze obroty uzywana w algorytmie Korfa.
class KorfMoveTable_ECP : public KorfMoveTable 
{

  private:

    bool first;

    
  public:

    KorfMoveTable_ECP(bool first) 
        : KorfMoveTable(KorfCube::EDGE_HALF_COMBINATION_NUMBER * KorfCube::EDGE_HALF_PERMUTATION_NUMBER),
          first(first)
    {}

    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName()
        { return ( first ? "KR_E1CP" : "KR_E2CP" ); }
    
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription()
        { return ( first ? "Edge first half choice and permutation coordinate move table" : "Edge second half choice and permutation coordinate move table" ); }


  protected:

    int GetCoord()
    { 
        int coords[3];
        pKCube->FromCubieCube(*pCube);
        pKCube->ToCoords(coords);
        return KorfCube::EDGE_COMBINATION(coords[1 + (first ? 0 : 1)])
             + KorfCube::EDGE_PERMUTATION(coords[1 + (first ? 0 : 1)]) * KorfCube::EDGE_HALF_COMBINATION_NUMBER;
    }
    
    void SetCoord(int coord)
    { 
        int coords[3];
        coords[0] = 0;
        coords[1 + (first ? 0 : 1)] = KorfCube::EDGE(coord % KorfCube::EDGE_HALF_COMBINATION_NUMBER,
            coord / KorfCube::EDGE_HALF_COMBINATION_NUMBER, 0);
        coords[2 - (first ? 0 : 1)] = KorfCube::EDGE(KorfCube::EDGE_HALF_COMBINATION_NUMBER - (coord % KorfCube::EDGE_HALF_COMBINATION_NUMBER) - 1,
            0, 0);
        pKCube->FromCoords(coords); 
        pKCube->ToCubieCube(*pCube);
    }

};

#endif // KorfMoveTable_ECP_h

