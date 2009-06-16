/////////////////////////////////////////////////////////
// Nazwa: KociembaMoveTable.h
// Opis: Klasa abstrakcyjna KociembaMoveTable reprezentujaca tablice
//       przeksztalcen stanow kostki Kociemby. 
/////////////////////////////////////////////////////////


#ifndef KociembaMoveTable_h
#define KociembaMoveTable_h

#include "../Cube/CubeMoveTable.h"
#include "KociembaCube.h"


// Klasa bazowa tablicy przeksztalcen dla algorytmu Kociemby.
class KociembaMoveTable : public CubeMoveTable 
{
      
  protected:
          
    int iCoordNum;

    // lokalny obiekt kostki Kociemby uzywany do przeksztalcen
    KociembaCube *pKCube;


  public:

    // Konstruktor. Wywoluje konstruktor klasy bazowej, ktory alokuje odpowiednia 
    // ilosc pamieci na tablice i ustawia pole pCube na podstawie argumentu.
    // Dodatkowo konstruktor przypisuje wskaznik do kostki Kociemby oraz ustawia,
    // czy tabela dotyczy drugiej fazy algorytmu.
    KociembaMoveTable(int coordNum) 
        throw (KociembaException)
        : CubeMoveTable(GetCoordSize(coordNum)),
          iCoordNum( coordNum >= 3 ? coordNum - 3 : coordNum ),
          pKCube(NULL)
    {}

    // Konstruktor kopiujacy.
    KociembaMoveTable(const KociembaMoveTable& table)
        : CubeMoveTable(*((CubeMoveTable*) &table)),
          iCoordNum(table.iCoordNum),
          pKCube(NULL)
    {}

    // Destruktor.
    ~KociembaMoveTable()
    {
        if (pKCube != NULL) delete pKCube;
    }

    // Operator przypisania.
    KociembaMoveTable& operator=(const KociembaMoveTable& table)
    {
        CubeMoveTable::operator=(*((CubeMoveTable*) &table));
        iCoordNum = table.iCoordNum;
        pKCube = NULL;
        return *this;
    }


  protected:
            
    virtual int GetCoord()
    { 
        int coords[3];
        pKCube->FromCubieCube(*pCube);
        pKCube->ToCoords(coords);
        return coords[iCoordNum];
    }
    
    virtual void SetCoord(int coord)
    { 
        int coords[3];
        pKCube->SetInitState();
        pKCube->ToCoords(coords);
        coords[iCoordNum] = coord;
        pKCube->FromCoords(coords); 
        pKCube->ToCubieCube(*pCube);
    }



  private:

    int GetCoordSize(int coordNum)
        throw (KociembaException);
/*    {
        switch (coordNum)
        {
            case 0: return KociembaCube::CORNER_ORIENTATION_NUMBER;
            case 1: return KociembaCube::EDGE_SLICE_COMBINATION_NUMBER;
            case 2: return KociembaCube::EDGE_ORIENTATION_NUMBER;
            case 3: return KociembaCube::CORNER_PERMUTATION_NUMBER;
            case 4: return KociembaCube::EDGE_SLICE_PERMUTATION_NUMBER;
            case 5: return KociembaCube::EDGE_TWO_SLICES_PERMUTATION_NUMBER;
        }
        
        // invalid argument
        throw KociembaException(KociembaException::ERR_MOVETABLE_COORD_INVALID);
    }
*/
};


#endif // KociembaMoveTable_h

