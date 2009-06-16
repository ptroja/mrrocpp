/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase1Solver.h
// Opis: Klasa KociembaPhase1Solver rozwiazujaca kostke zgodnie
//      z algorymem Kociembaa.
/////////////////////////////////////////////////////////

#ifndef KociembaPhase1Solver_h
#define KociembaPhase1Solver_h

#include "KociembaCube.h"
#include "KociembaPhase1PruningTable.h"
#include "../Cube/CubeSolver.h"

//#include "EdgeHalfPruningTable.h"


// Klasa znajdujaca rozwiazanie kostki algorytmem dwufazowym Kociemby.
class KociembaPhase1Solver : public CubeSolver 
{

  private:

    // Znacznik inicjalizacji tabel.
    static bool bInitialized;
          
    // Tabela odleglosci czesciowych stanow zlozonych ze wspolrzednych
    // orientacji i permutacji rogow od rozwiazania
    static KociembaPhase1PruningTable *ptCOnEMC;

    // Tabela odleglosci czesciowych stanow zlozonych ze wspolrzednych
    // orientacji i kombinacji pierwszej polowy krawedzi od rozwiazania
    static KociembaPhase1PruningTable *ptCOnEO;

    // Tabela odleglosci czesciowych stanow zlozonych ze wspolrzednych
    // orientacji i kombinacji drugiej polowy krawedzi od rozwiazania
    static KociembaPhase1PruningTable *ptEMCnEO;


  protected: 
    
    // Funkcja heurystyczna
    virtual int Heuristic(Cube* cube);
  
    // Inicjalizuje tablice przeksztalcen i odleglosci.
    void Init();


  public:

    // Konstruktor domyslny.    
    KociembaPhase1Solver() : CubeSolver() {}
    
    // Destruktor.
    virtual ~KociembaPhase1Solver() {}
    
    // Inicjalizuje tablice przeksztalcen i odleglosci.
    static void SInit();
    static void SClear();

    // Znajduje rozwiazanie kostki. Zwraca kod sukcesu lub porazki. Ustawia 
    // kod bledu i kod ostrzezenia.
    CubeSolution* FindSolution(Cube* scrambledCube)
        throw (Exception);

    CubeSolution* FindNextSolution(Cube* scrambledCube, const CubeSolution& solution)
        throw (Exception);

};


#endif // KociembaPhase1Solver_h

