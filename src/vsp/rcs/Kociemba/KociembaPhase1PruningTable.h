/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase1PruningTable.h
// Opis: Klasa KociembaPhase1PruningTable 
/////////////////////////////////////////////////////////

#ifndef KociembaPhase1PruningTable_h
#define KociembaPhase1PruningTable_h

#include "KociembaPruningTable.h"
#include "KociembaPhase1Cube.h"
#include "KociembaPhase1MoveTable.h"


// Klasa odleglosci czesciowych stanow kostki od rozwiazania. Klasa konstruowana
// jest na podstawie dwoch tablic przeksztalcen. Przechowuje odleglosc od 
// rozwiazania dla kazdej kombinacji wspolrzednych z obu tabel.
class KociembaPhase1PruningTable : public KociembaPruningTable
{

  public:
 
    KociembaPhase1PruningTable(int coord1Num, int coord2Num)
        : KociembaPruningTable(coord1Num, coord2Num)
    {}

    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName()
    {
        if (iCoord1Num==0 && iCoord2Num==1)
            return "KC_COnEMC";
        if (iCoord1Num==0 && iCoord2Num==2)
            return "KC_COnEO";
        if (iCoord1Num==1 && iCoord2Num==2)
            return "KC_EMCnEO";

        // invalid argument
        throw KociembaException(KociembaException::ERR_PRUNTABLE_COORD_INVALID);
    }
            
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription()
    {
        if (iCoord1Num==0 && iCoord2Num==1)
            return "Corner orientation and middle-slice edge combination coordinates pruning table";
        if (iCoord1Num==0 && iCoord2Num==2)
            return "Corner orientation and edge orientation coordinates pruning table";
        if (iCoord1Num==1 && iCoord2Num==2)
            return "Middle-slice edge combination and edge orientation coordinates pruning table";

        // invalid argument
        throw KociembaException(KociembaException::ERR_PRUNTABLE_COORD_INVALID);
    }


  protected:
    
    // Pobiera wspolrzedna z lokalnego obiektu kostki pCube.
    virtual int GetInitCoord()
    {
        int coordVal, coordVals[3];
        KociembaPhase1Cube cube;
        cube.ToCoords(coordVals);
        CCoordsToPTCoord(coordVal, coordVals[iCoord1Num], coordVals[iCoord2Num]);
        return coordVal;
    }

    // Wykonuje operacje przed generowaniem tablicy.
    void PreGenerate()
    {
        KociembaPruningTable::PreGenerate();
        mt1 = new KociembaPhase1MoveTable(iCoord1Num);
        mt2 = new KociembaPhase1MoveTable(iCoord2Num);
        mt1->Init();
        mt2->Init();
    }

    // Wykonuje operacje po wygenerowaniu tablicy.
    void PostGenerate()
    {
        delete mt1;
        delete mt2;
        mt1 = NULL;
        mt2 = NULL;
        KociembaPruningTable::PostGenerate();
    }


  private:

    inline void PTCoordToCCoords(int& coordNum, int& coord1Num, int& coord2Num)
    {
        coord1Num = coordNum % iCoord1Size;
        coord2Num = coordNum / iCoord1Size;
    }
    
    inline void CCoordsToPTCoord(int& coordNum, int& coord1Num, int& coord2Num)
    {
        coordNum = coord1Num + coord2Num * iCoord1Size;
    }

};


#endif // KociembaPhase1PruningTable_h
