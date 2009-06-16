/////////////////////////////////////////////////////////
// Nazwa: KorfPruningTable.h
// Opis: Klasa abstrakcyjna KorfPruningTable reprezentujaca tablice
//       heurystyczna odleglosci czesciowych stanow kostki od 
//       rozwiazania.
/////////////////////////////////////////////////////////


#ifndef KorfPruningTable_h
#define KorfPruningTable_h

#include "../Cube/CubePruningTable.h"
#include "KorfMoveTable_CO.h"
#include "KorfMoveTable_CP.h"
#include "KorfMoveTable_ECO.h"
#include "KorfMoveTable_ECP.h"


// Klasa odleglosci czesciowych stanow kostki od rozwiazania. Klasa konstruowana
// jest na podstawie dwoch tablic przeksztalcen. Przechowuje odleglosc od 
// rozwiazania dla kazdej kombinacji wspolrzednych z obu tabel.
class KorfPruningTable : public CubePruningTable
{

  private:
    
    int iCoordNum;


  protected:
            
    KorfMoveTable *mt1, *mt2;
      

  public:

    // Konstruktor. Deklaruje bazowe tablice przeksztalcen i poczatkowa
    // wartosc wspolrzednych z nimi zwiazanych.
    KorfPruningTable(int coordNum) 
        : CubePruningTable(GetCoordSize(coordNum)), 
          iCoordNum(coordNum), mt1(NULL), mt2(NULL)
    {}

    // Konstruktor kopiujacy.
    KorfPruningTable(const KorfPruningTable& table) 
        : CubePruningTable(*((CubePruningTable*) &table)),
          iCoordNum(table.iCoordNum), mt1(NULL), mt2(NULL)
    {}

    // Operator przypisania.
    KorfPruningTable& operator=(const KorfPruningTable& table) 
    {
        CubePruningTable::operator=(*((CubePruningTable*) &table));
        iCoordNum = table.iCoordNum;
        mt1 = NULL;
        mt2 = NULL;
        return *this;
    }

    // Zwraca nazwe pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetName()
    {
        switch (iCoordNum)
        {
            case 0: return "KR_C";
            case 1: return "KR_EFH";
            case 2: return "KR_ESH";
        }
        return ""; // throw Exc
    }
    
    // Zwraca opis danych tabeli.
    virtual const char* GetDescription()
    {
        switch (iCoordNum)
        {
            case 0: return "Corner orientation and permutation coordinates pruning table";
            case 1: return "Edge first half coordinates pruning table";
            case 2: return "Edge second half coordinates pruning table";
        }
        return ""; // throw Exc
    }


  protected:

    // Pobiera wspolrzedna z lokalnego obiektu kostki pCube.
    virtual int GetInitCoord()
    {
        int coords[3];
        KorfCube cube;
        cube.ToCoords(coords);
        return coords[iCoordNum];
    }


  private:
          
    static int GetCoordSize(int coordNum)
    {
        switch (coordNum)
        {
            case 0: return KorfCube::CORNER_COORD_NUMBER;
            case 1: return KorfCube::EDGE_HALF_COORD_NUMBER;
            case 2: return KorfCube::EDGE_HALF_COORD_NUMBER;
        }
        return 0; // throw Exc
    }

};


#endif // KorfPruningTable_h
