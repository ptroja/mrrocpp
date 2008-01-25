/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase1MoveTable.h
// Opis: Klasa KociembaMoveTable_CO reprezentujaca tablice
//       przeksztalcen permutacji rogow kostki Korfa. 
/////////////////////////////////////////////////////////

#ifndef KociembaPhase1MoveTable_h
#define KociembaPhase1MoveTable_h

#include "KociembaMoveTable.h"
#include "KociembaPhase1Cube.h"


// Klasa tablicy przeksztalcen wspolrzednych permutacji rogow przez podstawowe
// obroty grupy G1. Tablica uzywana w drugiej fazie algorytmu Kociemby.
class KociembaPhase1MoveTable : public KociembaMoveTable 
{

  public:
 
    KociembaPhase1MoveTable(int coordNum) 
        throw (KociembaException)
        : KociembaMoveTable(coordNum)
    {}

    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName()
    {
        switch (iCoordNum)
        {
            case 0: return "KC_CO";
            case 1: return "KC_EO";
            case 2: return "KC_EMC";
        }

        // invalid argument
        throw KociembaException(KociembaException::ERR_MOVETABLE_COORD_INVALID);
    }
    
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription()
    {
        switch (iCoordNum)
        {
            case 0: return "Corner orientataion coordinate move table";
            case 1: return "Edge orientation coordinate move table";
            case 2: return "Edge middle-slice choice coordinate move table";
        }

        // invalid argument
        throw KociembaException(KociembaException::ERR_MOVETABLE_COORD_INVALID);
    }


  protected:
    
    // Wykonuje operacje przed generowaniem tablicy.
    void PreGenerate()
    {
        KociembaMoveTable::PreGenerate();
        pKCube = new KociembaPhase1Cube();
    }

    // Wykonuje operacje po wygenerowaniu tablicy.
    void PostGenerate()
    {
        delete pKCube;
        pKCube = NULL;
        KociembaMoveTable::PostGenerate();
    }

};


#endif // KociembaPhase1MoveTable_h

