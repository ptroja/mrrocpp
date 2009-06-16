/////////////////////////////////////////////////////////
// Nazwa: KociembaPruningTable.h
// Opis: Klasa KociembaPruningTable 
/////////////////////////////////////////////////////////

#ifndef KociembaPruningTable_h
#define KociembaPruningTable_h

#include "../Cube/CubePruningTable.h"
#include "KociembaMoveTable.h"


// Klasa odleglosci czesciowych stanow kostki od rozwiazania. Klasa konstruowana
// jest na podstawie dwoch tablic przeksztalcen. Przechowuje odleglosc od 
// rozwiazania dla kazdej kombinacji wspolrzednych z obu tabel.
class KociembaPruningTable : public CubePruningTable
{

  protected:
    
    int iCoord1Num;
    int iCoord2Num;

    int iCoord1Size;
    int iCoord2Size;
    
    KociembaMoveTable *mt1, *mt2;
      

  public:

    // Konstruktor. Deklaruje bazowe tablice przeksztalcen i poczatkowa
    // wartosc wspolrzednych z nimi zwiazanych.
    KociembaPruningTable(int coord1Num, int coord2Num) 
        : CubePruningTable(GetCoordSize(coord1Num) * GetCoordSize(coord2Num)),
          iCoord1Num( coord1Num >= 3 ? coord1Num - 3 : coord1Num ),
          iCoord2Num( coord2Num >= 3 ? coord2Num - 3 : coord2Num ),
          iCoord1Size(GetCoordSize(coord1Num)),
          iCoord2Size(GetCoordSize(coord2Num)),
          mt1(NULL), mt2(NULL)
    {}

    // Konstruktor kopiujacy.
    KociembaPruningTable(const KociembaPruningTable& table)
        : CubePruningTable(*((CubePruningTable*) &table)),
          iCoord1Num(table.iCoord1Num),
          iCoord2Num(table.iCoord2Num),
          iCoord1Size(table.iCoord1Size),
          iCoord2Size(table.iCoord2Size),
          mt1(NULL), mt2(NULL)
    {}
    
    // Konstruktor kopiujacy.
    KociembaPruningTable& operator=(const KociembaPruningTable& table)
    {
        if (this!=&table)
        {
            CubePruningTable::operator=(*((CubePruningTable*)&table));
            iCoord1Num = table.iCoord1Num;
            iCoord2Num = table.iCoord2Num;
            iCoord1Size = table.iCoord1Size;
            iCoord2Size = table.iCoord2Size;
            mt1 = NULL;
            mt2 = NULL;
        }
        return *this;
    }

    // Udostepnienie wartosci z pola o indeksie index.
    int Get(int coord1, int coord2)
    {
        int coordVal;
        CCoordsToPTCoord(coordVal, coord1, coord2);
        return CubePruningTable::Get(coordVal);
    }


  protected:
            
    // Ustawia stan lokalnego obiektu kostki pCube zgodnie ze wspolrzedna coord.
    virtual int GetCoord(int coordVal, CubeMove::eMove move, CubeMove::eTurn turn)
    {
        int coord1Val, coord2Val;
        PTCoordToCCoords(coordVal, coord1Val, coord2Val);
        for (int i=0; i<=turn; i+=mt1->GetTurn(move)+1)
        {
            coord1Val = mt1->Get(coord1Val, move);   
            coord2Val = mt2->Get(coord2Val, move);   
        }
        CCoordsToPTCoord(coordVal, coord1Val, coord2Val);
        return coordVal;
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

    int GetCoordSize(int coordNum);

};


#endif // KociembaPruningTable_h
