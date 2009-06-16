/////////////////////////////////////////////////////////
// Nazwa: CubeMoveTable.h
// Opis: Klasa abstrakcyjna CubeMoveTable reprezentujaca tablice
//       przeksztalcen stanow kostki. 
/////////////////////////////////////////////////////////


#ifndef CubeMoveTable_h
#define CubeMoveTable_h

#include "../General/DataTable.h"
#include "CubieCube.h"
#include "CubeMove.h"


// Abstrakcyjna klasa bazowa dla klas tablic przeksztalcen. Dostarcza 
// mechanizm tworzenia. W klasach pochodnych musza zostac zdefiniowane
// metody dotyczace konwersji miedzy stanem kostki a jego wspolrzedna.
class CubeMoveTable : public DataTable
{
  private:
    
    int iCoordSize;

    
  protected:
    
    // Kostka, na ktorej wykonywane sa ruchy podczas generowania tabeli.
    CubieCube *pCube;
    
   
  public:
    
    // Konstruktor.
    CubeMoveTable(int coordSize);

    // Konstruktor kopiujacy.
    CubeMoveTable(const CubeMoveTable& table);

    // Destruktor.
    ~CubeMoveTable();
        
    // Konstruktor kopiujacy.
    CubeMoveTable& operator=(const CubeMoveTable& table);

    // Udostepnia wpis z tabeli okreslajacy przeksztalcenie wspolrzednej coord
    // przez ruch move. 
    int Get(int coord, CubeMove::eMove move);
    
    // Zwraca wielkosc tabeli.
    int GetSize()
        { return iCoordSize; }

    // Zwraca rozszerzenie pliku, w ktorym zapisana jest zawartosc tabeli.
    virtual const char* GetExtention()
        { return "mtb"; }

    // Zwraca domyslny obrot dla ruchu move.
    virtual CubeMove::eTurn GetTurn(CubeMove::eMove move);
    

protected:

    // Pobiera wspolrzedna z lokalnego obiektu kostki pCube.
    virtual int GetCoord() = 0;

    // Ustawia stan lokalnego obiektu kostki pCube zgodnie ze wspolrzedna coord.
    virtual void SetCoord(int coord) = 0;
    
    // Wykonuje operacje przed generowaniem tablicy.
    virtual void PreGenerate();

    // Wykonuje operacje po wygenerowaniu tablicy.
    virtual void PostGenerate();


private:
    
    // Generuje tabele. Dla kazdej wartosci wspolrzednej obliczane i zapisywane
    // do tabeli jest jej przeksztalcenie przez 6 podstawowych ruchow. 
    virtual void Generate();
    
    int GetDataCoord(int coord, CubeMove::eMove move);
    
};


#endif // CubeMoveTable_h

