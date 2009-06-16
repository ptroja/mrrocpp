/////////////////////////////////////////////////////////
// Nazwa: KorfMoveTable.h
// Opis: Klasa abstrakcyjna KorfMoveTable reprezentujaca tablice
//       przeksztalcen stanow kostki Korfa. 
/////////////////////////////////////////////////////////


#ifndef KorfMoveTable_h
#define KorfMoveTable_h

#include "../Cube/CubeMoveTable.h"
#include "KorfCube.h"


// Klasa bazowa tablicy przeksztalcen dla algorytmu Korfa.
class KorfMoveTable : public CubeMoveTable 
{
      
  protected:
      
    // lokalny obiekt kostki Korfa uzywany do przeksztalcen
    KorfCube *pKCube;
    
    
  public:

    // Konstruktor. Wywoluje konstruktor klasy bazowej, ktory alokuje odpowiednia 
    // ilosc pamieci na tablice i ustawia pole pCube na podstawie argumentu.
    // Dodatkowo konstruktor przypisuje wskaznik do kostki Korfa.
    KorfMoveTable(int coordSize) 
        : CubeMoveTable(coordSize),
          pKCube(NULL)
    {}

    // Konstruktor kopiujacy.
    KorfMoveTable(const KorfMoveTable& table) 
        : CubeMoveTable(*((CubeMoveTable*) &table)),
          pKCube(NULL)
    {}

    // Destruktor.
    ~KorfMoveTable() 
    {
        if (pKCube!= NULL) delete pKCube;
    }

    // Konstruktor kopiujacy.
    KorfMoveTable& operator=(const KorfMoveTable& table) 
    {
        CubeMoveTable::operator=(*((CubeMoveTable*) &table));
        pKCube = NULL;
        return *this;
    }


  protected:
  
    // Wykonuje operacje przed generowaniem tablicy.
    void PreGenerate()
    {
        CubeMoveTable::PreGenerate();
        pKCube = new KorfCube();
    }

    // Wykonuje operacje po wygenerowaniu tablicy.
    void PostGenerate()
    {
        delete pKCube;
        pKCube = NULL;
        CubeMoveTable::PostGenerate();
    } 

};


#endif // KorfMoveTable_h

