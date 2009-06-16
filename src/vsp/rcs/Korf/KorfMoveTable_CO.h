/////////////////////////////////////////////////////////
// Nazwa: KorfMoveTable_CO.h
// Opis: Klasa KorfMoveTable_CO reprezentujaca tablice
//       przeksztalcen orientacji rogow kostki Korfa. 
/////////////////////////////////////////////////////////

#ifndef KorfMoveTable_CO_h
#define KorfMoveTable_CO_h

#include "KorfMoveTable.h"


// Klasa tablicy przeksztalcen wspolrzednych orientacji rogow przez
// pojedyncze obroty uzywana w algorytmie Korfa.
class KorfMoveTable_CO : public KorfMoveTable 
{
  public:
    KorfMoveTable_CO() 
        : KorfMoveTable(KorfCube::CORNER_ORIENTATION_NUMBER)
    {}

    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName()
        { return "KR_CO"; }
    
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription()
        { return "Corner orientataion coordinate move table"; }


  protected:
    int GetCoord()
    { 
        int coords[3];
        pKCube->FromCubieCube(*pCube);
        pKCube->ToCoords(coords);
        return KorfCube::CORNER_ORIENTATION(coords[0]);
    }
    
    void SetCoord(int coord)
    { 
        int coords[3];
        coords[0] = KorfCube::CORNER(0, coord);
        coords[1] = KorfCube::EDGE(0,0,0);
        coords[2] = KorfCube::EDGE(KorfCube::EDGE_HALF_COMBINATION_NUMBER - 1,0,0);
        pKCube->FromCoords(coords); 
        pKCube->ToCubieCube(*pCube);
    }

};

#endif // KorfMoveTable_CO_h

