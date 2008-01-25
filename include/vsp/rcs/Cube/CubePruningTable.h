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


#endif // CubePruningTable_h
