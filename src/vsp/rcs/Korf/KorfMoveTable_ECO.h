/////////////////////////////////////////////////////////
// Nazwa: KorfMoveTable_ECO.h
// Opis: Klasa KorfMoveTable_ECO reprezentujaca tablice
//       przeksztalcen kombinacji i orientacji polowy
//       krawedzi kostki Korfa. 
/////////////////////////////////////////////////////////

#ifndef KorfMoveTable_ECO_h
#define KorfMoveTable_ECO_h

#include "KorfMoveTable.h"


// Klasa tablicy przeksztalcen wspolrzednych orientacji i kombinacji 
// pierwszej polowy krawedzi przez pojedyncze obroty.
class KorfMoveTable_ECO : public KorfMoveTable 
{

  private:

    bool first;
    

  public:

    KorfMoveTable_ECO(bool first) 
        : KorfMoveTable(KorfCube::EDGE_HALF_COMBINATION_NUMBER * KorfCube::EDGE_HALF_ORIENTATION_NUMBER),
          first(first)
    {}

    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName()
        { return ( first ? "KR_E1CO" : "KR_E2CO" ); }
    
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription()
        { return ( first ? "Edge first half choice and orientation coordinate move table" : "Edge second half choice and orientation coordinate move table" ); }


  protected:
    int GetCoord()
    { 
        int coords[3];
        pKCube->FromCubieCube(*pCube);
        pKCube->ToCoords(coords);
        return KorfCube::EDGE_COMBINATION(coords[1 + (first ? 0 : 1)])
             + KorfCube::EDGE_ORIENTATION(coords[1 + (first ? 0 : 1)]) * KorfCube::EDGE_HALF_COMBINATION_NUMBER;
    }
    
    void SetCoord(int coord)
    { 
        int coords[3];
        coords[0] = 0;
        coords[1 + (first ? 0 : 1)] = KorfCube::EDGE(coord % KorfCube::EDGE_HALF_COMBINATION_NUMBER,
            0, coord / KorfCube::EDGE_HALF_COMBINATION_NUMBER);
        coords[2 - (first ? 0 : 1)] = KorfCube::EDGE(KorfCube::EDGE_HALF_COMBINATION_NUMBER - (coord % KorfCube::EDGE_HALF_COMBINATION_NUMBER) - 1,
            0, 0);
        pKCube->FromCoords(coords); 
        pKCube->ToCubieCube(*pCube);
    }

};

#endif // KorfMoveTable_ECO_h
