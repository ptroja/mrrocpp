/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase2MoveTable.h
// Opis: Klasa KociembaMoveTable_CP reprezentujaca tablice
//       przeksztalcen permutacji rogow kostki Korfa. 
/////////////////////////////////////////////////////////

#ifndef KociembaPhase2MoveTable_h
#define KociembaPhase2MoveTable_h

#include "KociembaMoveTable.h"
#include "KociembaPhase2Cube.h"


// Klasa tablicy przeksztalcen wspolrzednych permutacji rogow przez podstawowe
// obroty grupy G1. Tablica uzywana w drugiej fazie algorytmu Kociemby.
class KociembaPhase2MoveTable : public KociembaMoveTable 
{

  public:

    KociembaPhase2MoveTable(int coordNum) 
        throw (KociembaException)
        : KociembaMoveTable(coordNum + 3)
    {}

    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName()
    {
        switch (iCoordNum)
        {
            case 0: return "KC_CP";
            case 1: return "KC_EMP";
            case 2: return "KC_ENP";
        }

        // invalid argument
        throw KociembaException(KociembaException::ERR_MOVETABLE_COORD_INVALID);
    }
    
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription()
    {
        switch (iCoordNum)
        {
            case 0: return "Corner permutation coordinate move table";
            case 1: return "Edge middle-slice permutation coordinate move table";
            case 2: return "Edge non-middle-slice permutation coordinate move table";
        }

        // invalid argument
        throw KociembaException(KociembaException::ERR_MOVETABLE_COORD_INVALID);
    }


  protected:
    
    // Wykonuje operacje przed generowaniem tablicy.
    void PreGenerate()
    {
        KociembaMoveTable::PreGenerate();
        pKCube = new KociembaPhase2Cube();
    }

    // Wykonuje operacje po wygenerowaniu tablicy.
    void PostGenerate()
    {
        delete pKCube;
        pKCube = NULL;
        KociembaMoveTable::PostGenerate();
    }

    // Zwraca domyslny obrot dla ruchu move.
    CubeMove::eTurn GetTurn(CubeMove::eMove move)
    {
        // w fazie drugiej nie wszystkie ruchy sa dozwolone
        if (move != CubeMove::U && move != CubeMove::D) 
            return CubeMove::HALF;
        
        return KociembaMoveTable::GetTurn(move);
    }    
    
};


#endif // KociembaPhase2MoveTable_h
