/////////////////////////////////////////////////////////
// Nazwa: KorfSolver.h
// Opis: Klasa KorfSolver rozwiazujaca kostke zgodnie
//      z algorymem Korfa.
/////////////////////////////////////////////////////////

#ifndef KorfSolver_h
#define KorfSolver_h

#include "KorfCube.h"
#include "KorfPruningTable.h"
#include "../Cube/CubeSolver.h"

//#include "EdgeHalfPruningTable.h"


// Klasa znajdujaca rozwiazanie kostki algorytmem dwufazowym Kociemby.
class KorfSolver : public CubeSolver 
{

  private:

    // Znacznik inicjalizacji tabel.
    static bool bInitialized;
          
    // Tabela odleglosci czesciowych stanow zlozonych ze wspolrzednych
    // orientacji i permutacji rogow od rozwiazania
    static KorfPruningTable *ptCorner;

    // Tabela odleglosci czesciowych stanow zlozonych ze wspolrzednych
    // orientacji i kombinacji pierwszej polowy krawedzi od rozwiazania
    static KorfPruningTable *ptEdgeFirstHalf;

    // Tabela odleglosci czesciowych stanow zlozonych ze wspolrzednych
    // orientacji i kombinacji drugiej polowy krawedzi od rozwiazania
    static KorfPruningTable *ptEdgeSecondHalf;


  protected: 
    
    // Funkcja heurystyczna
    virtual int Heuristic(Cube* cube);
  
    // Inicjalizuje tablice przeksztalcen i odleglosci.
    void Init();


  public:

    // Konstruktor domyslny.    
    KorfSolver() : CubeSolver() {}
    
    // Destruktor.
    virtual ~KorfSolver() {}
    
    // Inicjalizuje tablice przeksztalcen i odleglosci.
    static void SInit();
    static void SClear();

    CubeSolution* FindSolution(Cube* scrambledCube)
        throw (Exception);


};


#endif // KorfSolver_h

