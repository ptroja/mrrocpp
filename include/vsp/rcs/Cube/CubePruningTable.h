/////////////////////////////////////////////////////////
// Nazwa: CubePruningTable.h
// Opis: Klasa abstrakcyjna CubePruningTable reprezentujaca tablice
//       heurystyczna odleglosci czesciowych stanow kostki od 
//       rozwiazania.
/////////////////////////////////////////////////////////


#ifndef CubePruningTable_h
#define CubePruningTable_h

#include "../General/DataTable.h"
#include "CubeMove.h"


// Klasa odleglosci czesciowych stanow kostki od rozwiazania. Klasa konstruowana
// jest na podstawie dwoch tablic przeksztalcen. Przechowuje odleglosc od 
// rozwiazania dla kazdej kombinacji wspolrzednych z obu tabel.
class CubePruningTable : public DataTable
{
      
  private:
          
    int iCoordSize;

    const static int MAX_DEPTH;
    
    
  protected:
    
    //Cube *pCube;
    

  public:

    // Konstruktor.
    CubePruningTable(int coordSize);
    
    // Destruktor.
    ~CubePruningTable();
    
    // Udostepnienie wartosci z pola o indeksie index.
    int Get(int coord);
    

  protected:

    // Pobiera wspolrzedna z lokalnego obiektu kostki pCube.
    virtual int GetInitCoord() = 0;

    // Ustawia stan lokalnego obiektu kostki pCube zgodnie ze wspolrzedna coord.
    virtual int GetCoord(int coord, CubeMove::eMove move, CubeMove::eTurn turn) = 0;
    
    // Sprawdza czy dozwolony jest ruch.
    virtual bool IsAllowed(CubeMove::eMove move, CubeMove::eTurn turn)
    { return true; }

    // Wykonuje operacje przed generowaniem tablicy.
    virtual void PreGenerate();

    // Wykonuje operacje po wygenerowaniu tablicy.
    virtual void PostGenerate();

    // Zwraca rozszerzenie pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetExtention()
        { return "ptb"; }


  private:
    
    // Generuje tabele. Dla kazdej wartosci wspolrzednej obliczane i zapisywane
    // do tabeli jest jej przeksztalcenie przez 6 podstawowych ruchow. 
    virtual void Generate();
    
    // Udostepnienie wartosci z pola o indeksie index.
    void Set(int coord, int value);
    
    int GetDataCoord(int coord);
    int GetInDataPos(int coord);
    
};

////////////////////////////////////////////////////////////////////////////////
// DATA ACCESS INLINE METHODS
////////////////////////////////////////////////////////////////////////////////

inline int CubePruningTable::Get(int coord) 
{
    // Gets data from table and chooses only part of data relevant for coord
    return ( ((char*) iaTable)[coord/2] >> (4*(coord%2))) & 0x0f;
}
    
inline void CubePruningTable::Set(int coord, int value) 
{
    // Gets data from table and updates only part of data relevant for coord
    ((char*) iaTable)[coord/2] = ( ((char*) iaTable)[coord/2] & ~(0x0f << ((4*(coord%2))))) 
             | ( (value & 0x0f) << ((4*(coord%2))) );
    
}

inline int CubePruningTable::GetDataCoord(int coord)
{
    return coord / (sizeof(int) / sizeof(char) * 2);
}    

inline int CubePruningTable::GetInDataPos(int coord)
{
    return 4 * coord % (sizeof(int) / sizeof(char) * 2);
}    

#endif // CubePruningTable_h
