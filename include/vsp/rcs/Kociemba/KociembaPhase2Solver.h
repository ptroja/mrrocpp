/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase2Solver.h
// Opis: Klasa KociembaPhase2Solver rozwiazujaca kostke zgodnie
//      z algorymem Kociembaa.
/////////////////////////////////////////////////////////

#ifndef KociembaPhase2Solver_h
#define KociembaPhase2Solver_h

#include "KociembaCube.h"
#include "KociembaPhase2PruningTable.h"
#include "../Cube/CubeSolver.h"

//#include "EdgeHalfPruningTable.h"


// Klasa znajdujaca rozwiazanie kostki algorytmem dwufazowym Kociemby.
class KociembaPhase2Solver : public CubeSolver 
{

  private:

    // Znacznik inicjalizacji tabel.
    static bool bInitialized;
          
    // Tabela odleglosci czesciowych stanow zlozonych ze wspolrzednych
    // orientacji i permutacji rogow od rozwiazania
    static KociembaPhase2PruningTable *ptCPnEMP;

    // Tabela odleglosci czesciowych stanow zlozonych ze wspolrzednych
    // orientacji i kombinacji pierwszej polowy krawedzi od rozwiazania
    static KociembaPhase2PruningTable *ptEMPnENP;


  protected: 
    
    // Funkcja heurystyczna
    virtual int Heuristic(Cube* cube);
  
    // Inicjalizuje tablice przeksztalcen i odleglosci.
    void Init();


  public:

    // Konstruktor domyslny.    
    KociembaPhase2Solver() : CubeSolver() {}
    
    // Destruktor.
    virtual ~KociembaPhase2Solver() {}
    
    // Inicjalizuje tablice przeksztalcen i odleglosci.
    static void SInit();
    static void SClear();

    // Znajduje rozwiazanie kostki. Zwraca kod sukcesu lub porazki. Ustawia 
    // kod bledu i kod ostrzezenia.
    CubeSolution* FindSolution(Cube* scrambledCube)
        throw (Exception);

};


#endif // KociembaPhase2Solver_h

