/////////////////////////////////////////////////////////
// Nazwa: KorfPruningTable_Edge.h
// Opis: Klasa abstrakcyjna KorfPruningTable reprezentujaca tablice
//       heurystyczna odleglosci czesciowych stanow kostki od 
//       rozwiazania.
/////////////////////////////////////////////////////////


#ifndef KorfPruningTable_Edge_h
#define KorfPruningTable_Edge_h

#include "KorfPruningTable.h"
#include "KorfMoveTable_ECO.h"
#include "KorfMoveTable_ECP.h"


// Klasa odleglosci czesciowych stanow kostki od rozwiazania. Klasa konstruowana
// jest na podstawie dwoch tablic przeksztalcen. Przechowuje odleglosc od 
// rozwiazania dla kazdej kombinacji wspolrzednych z obu tabel.
class KorfPruningTable_Edge : public KorfPruningTable
{

  private:

    bool bFirst;
    

  public:

    // Konstruktor. Deklaruje bazowe tablice przeksztalcen i poczatkowa
    // wartosc wspolrzednych z nimi zwiazanych.
    KorfPruningTable_Edge(bool first) 
        : KorfPruningTable(first?1:2), bFirst(first)
    {}

    // Konstruktor kopiujacy.
    KorfPruningTable_Edge(const KorfPruningTable_Edge& table) 
        : KorfPruningTable(*((KorfPruningTable*) &table)), bFirst(table.bFirst)
    {}

    // Operator przypisania.
    KorfPruningTable_Edge& operator=(const KorfPruningTable_Edge& table) 
    {
        KorfPruningTable::operator=(*((KorfPruningTable*) &table));
        bFirst = table.bFirst;
        return *this;
    }


  protected:

    // Ustawia stan lokalnego obiektu kostki pCube zgodnie ze wspolrzedna coord.
    virtual int GetCoord(int coord, CubeMove::eMove move, CubeMove::eTurn turn)
    {
        int coord1Val, coord2Val;
        int tmpCoord1, tmpCoord2, tmpCoord3;
        tmpCoord1 = KorfCube::EDGE_COMBINATION(coord);
        tmpCoord2 = KorfCube::EDGE_PERMUTATION(coord);
        tmpCoord3 = KorfCube::EDGE_ORIENTATION(coord);
        coord1Val = tmpCoord1 + tmpCoord2 * KorfCube::EDGE_HALF_COMBINATION_NUMBER;
        coord2Val = tmpCoord1 + tmpCoord3 * KorfCube::EDGE_HALF_COMBINATION_NUMBER;
        for (int i=0; i<=turn; i++)
        {
            coord1Val = mt1->Get(coord1Val, move);   
            coord2Val = mt2->Get(coord2Val, move);   
        }
        tmpCoord1 = coord1Val % KorfCube::EDGE_HALF_COMBINATION_NUMBER;
        tmpCoord2 = coord1Val / KorfCube::EDGE_HALF_COMBINATION_NUMBER;
        tmpCoord3 = coord2Val / KorfCube::EDGE_HALF_COMBINATION_NUMBER;
        return KorfCube::EDGE(tmpCoord1, tmpCoord2, tmpCoord3);
    }
    
    // Wykonuje operacje przed generowaniem tablicy.
    virtual void PreGenerate()
    {
        CubePruningTable::PreGenerate();
        mt1 = new KorfMoveTable_ECP(bFirst);
        mt2 = new KorfMoveTable_ECO(bFirst);
        mt1->Init();
        mt2->Init();
    }

    // Wykonuje operacje po wygenerowaniu tablicy.
    virtual void PostGenerate()
    {
        delete mt1;
        delete mt2;
        mt1 = NULL;
        mt2 = NULL;
        CubePruningTable::PostGenerate();
    }

};


#endif // KorfPruningTable_Edge_h
