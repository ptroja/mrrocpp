/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase2PruningTable.h
// Opis: Klasa KociembaPhase2PruningTable 
/////////////////////////////////////////////////////////

#ifndef KociembaPhase2PruningTable_h
#define KociembaPhase2PruningTable_h

#include "KociembaPruningTable.h"
#include "KociembaPhase2Cube.h"
#include "KociembaPhase2MoveTable.h"


// Klasa odleglosci czesciowych stanow kostki od rozwiazania. Klasa konstruowana
// jest na podstawie dwoch tablic przeksztalcen. Przechowuje odleglosc od 
// rozwiazania dla kazdej kombinacji wspolrzednych z obu tabel.
class KociembaPhase2PruningTable : public KociembaPruningTable
{

  public:

    KociembaPhase2PruningTable(int coord1Num, int coord2Num) 
        : KociembaPruningTable(coord1Num + 3, coord2Num + 3)
    {}

    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName()
    {
        if (iCoord1Num==0 && iCoord2Num==1)
            return "KC_CPnEMP";
        if (iCoord1Num==1 && iCoord2Num==2)
            return "KC_EMPnENP";

        // invalid argument
        throw KociembaException(KociembaException::ERR_PRUNTABLE_COORD_INVALID);
    }
            
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription()
    {
        if (iCoord1Num==0 && iCoord2Num==1)
            return "Corner permutation and middle-slice edge permutation coordinates pruning table";
        if (iCoord1Num==1 && iCoord2Num==2)
            return "Middle-slice edge permutation and non middle-slice edge permutation coordinates pruning table";

        // invalid argument
        throw KociembaException(KociembaException::ERR_PRUNTABLE_COORD_INVALID);
    }


  protected:
    
    // Pobiera wspolrzedna z lokalnego obiektu kostki pCube.
    virtual int GetInitCoord()
    {
        int coordVal, coordVals[3];
        KociembaPhase2Cube cube;
        cube.ToCoords(coordVals);
        CCoordsToPTCoord(coordVal, coordVals[iCoord1Num], coordVals[iCoord2Num]);
        return coordVal;
    }

    // Wykonuje operacje przed generowaniem tablicy.
    void PreGenerate()
    {
        KociembaPruningTable::PreGenerate();
        mt1 = new KociembaPhase2MoveTable(iCoord1Num);
        mt2 = new KociembaPhase2MoveTable(iCoord2Num);
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

    bool IsAllowed(CubeMove::eMove move, CubeMove::eTurn turn)
    {
        if (move != CubeMove::U && move != CubeMove::D && turn != CubeMove::HALF)
            return false;
        return KociembaPruningTable::IsAllowed(move, turn);
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


#endif // KociembaPhase2PruningTable_h
