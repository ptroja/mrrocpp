/////////////////////////////////////////////////////////
// Nazwa: KociembaSolver.h
// Opis: Klasa KociembaSolver rozwiazujaca kostke zgodnie
//      z algorymem Kociembaa.
/////////////////////////////////////////////////////////

#ifndef KociembaSolver_h
#define KociembaSolver_h

#include "../Cube/CubeSolver.h"
#include "../Cube/CubieCube.h"
#include "KociembaSolution.h"
#include "KociembaException.h"


// Klasa znajdujaca rozwiazanie kostki algorytmem dwufazowym Kociemby.
class KociembaSolver : CubeSolver
{

  public:

	virtual void Init() {}
	virtual int Heuristic(Cube*) { return 0; }

    // Znajduje rozwiazanie kostki. Zwraca kod sukcesu lub porazki. Ustawia 
    // kod bledu i kod ostrzezenia.
    static CubeSolution* FindSolution(Cube* scrambledCube)
           throw (Exception);

    static CubeSolution* FindNextSolution(Cube* scrambledCube, const KociembaSolution& solution)
        throw (Exception);

    static CubeSolution* FindOptimalSolution(Cube* scrambledCube)
           throw (Exception);

    // Inicjalizuje tablice przeksztalcen i odleglosci.
    static void SInit();

        
  private:
    static CubeSolution* GetPhase1Solution(const KociembaSolution& solution);
        
    static bool IsFullSolution(CubieCube* scrambledCube, const CubeSolution& solution)
        throw (Exception);

};


#endif // KociembaSolver_h
