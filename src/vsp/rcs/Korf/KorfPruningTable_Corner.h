/////////////////////////////////////////////////////////
// Nazwa: KorfPruningTable_Corner.h
// Opis: Klasa abstrakcyjna KorfPruningTable reprezentujaca tablice
//       heurystyczna odleglosci czesciowych stanow kostki od 
//       rozwiazania.
/////////////////////////////////////////////////////////


#ifndef KorfPruningTable_Corner_h
#define KorfPruningTable_Corner_h

#include "KorfPruningTable.h"
#include "KorfMoveTable_CO.h"
#include "KorfMoveTable_CP.h"


// Klasa odleglosci czesciowych stanow kostki od rozwiazania. Klasa konstruowana
// jest na podstawie dwoch tablic przeksztalcen. Przechowuje odleglosc od 
// rozwiazania dla kazdej kombinacji wspolrzednych z obu tabel.
class KorfPruningTable_Corner : public KorfPruningTable
{

  public:

    // Konstruktor. Deklaruje bazowe tablice przeksztalcen i poczatkowa
    // wartosc wspolrzednych z nimi zwiazanych.
    KorfPruningTable_Corner() 
        : KorfPruningTable(0)
    {}

    // Konstruktor kopiujacy.
    KorfPruningTable_Corner(const KorfPruningTable_Corner& table) 
        : KorfPruningTable(*((KorfPruningTable*) &table))
    {}

    // Operator przypisania.
    KorfPruningTable_Corner& operator=(const KorfPruningTable_Corner& table) 
    {
        KorfPruningTable::operator=(*((KorfPruningTable*) &table));
        return *this;
    }


  protected:

    // Ustawia stan lokalnego obiektu kostki pCube zgodnie ze wspolrzedna coord.
    virtual int GetCoord(int coord, CubeMove::eMove move, CubeMove::eTurn turn)
    {
        int coord1Val, coord2Val;
        coord1Val = KorfCube::CORNER_PERMUTATION(coord);
        coord2Val = KorfCube::CORNER_ORIENTATION(coord);
        for (int i=0; i<=turn; i+=mt1->GetTurn(move)+1)
        {
            coord1Val = mt1->Get(coord1Val, move);   
            coord2Val = mt2->Get(coord2Val, move);   
        }
        return KorfCube::CORNER(coord1Val, coord2Val);
    }
    
    // Wykonuje operacje przed generowaniem tablicy.
    virtual void PreGenerate()
    {
        CubePruningTable::PreGenerate();
        mt1 = new KorfMoveTable_CP();
        mt2 = new KorfMoveTable_CO();
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


#endif // KorfPruningTable_Corner_h
