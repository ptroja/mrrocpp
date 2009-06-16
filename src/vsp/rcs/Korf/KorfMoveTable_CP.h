/////////////////////////////////////////////////////////
// Nazwa: KorfMoveTable_CP.h
// Opis: Klasa KorfMoveTable_CP reprezentujaca tablice
//       przeksztalcen permutacji rogow kostki Korfa. 
/////////////////////////////////////////////////////////

#ifndef KorfMoveTable_CP_h
#define KorfMoveTable_CP_h

#include "KorfMoveTable.h"


// Klasa tablicy przeksztalcen wspolrzednych permutacji rogow przez pojedyncze
// obroty uzywana w algorytmie Korfa.
class KorfMoveTable_CP : public KorfMoveTable 
{

  public:

    KorfMoveTable_CP() 
        : KorfMoveTable(KorfCube::CORNER_PERMUTATION_NUMBER)
    {}

    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName()
        { return "KR_CP"; }
    
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription()
        { return "Corner permutation coordinate move table"; }


  protected:

    int GetCoord()
    { 
        int coords[3];
        pKCube->FromCubieCube(*pCube);
        pKCube->ToCoords(coords);
        return KorfCube::CORNER_PERMUTATION(coords[0]);
    }
    
    void SetCoord(int coord)
    { 
        int coords[3];
        coords[0] = KorfCube::CORNER(coord, 0);
        coords[1] = KorfCube::EDGE(0,0,0);
        coords[2] = KorfCube::EDGE(KorfCube::EDGE_HALF_COMBINATION_NUMBER - 1,0,0);
        pKCube->FromCoords(coords); 
        pKCube->ToCubieCube(*pCube);
    }

};

#endif // KorfMoveTable_CP_h

